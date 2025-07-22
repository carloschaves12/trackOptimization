#!/usr/bin/env python3
"""optimization.py  – versión *solo consola*

Uso típico
~~~~~~~~~~
    python optimization.py -t 2 -f direcciones.txt --json-out

• Lee todas las direcciones de un fichero de texto (una por línea).
• Optimiza rutas abiertas (no regresa al depósito) con OR‑Tools.
• Geocodificación robusta: Nominatim exacto + fuzzy + Photon.
• Muestra una tabla en consola y puede exportar JSON.

Requisitos
~~~~~~~~~~
    pip install ortools geopy requests rapidfuzz rich click
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Sequence

import click
import requests
from geopy import Nominatim
from geopy.extra.rate_limiter import RateLimiter
from geopy.geocoders import Photon
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from rapidfuzz import fuzz
from rich.console import Console
from rich.table import Table

console = Console()

# ──────────────────────────────────────────────────────────────────────────────
# Dataclass para almacenar coordenadas
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Location:
    address: str
    lat: float
    lon: float

# ──────────────────────────────────────────────────────────────────────────────
# Herramientas de geocodificación
# ──────────────────────────────────────────────────────────────────────────────

def _normalize(addr: str) -> str:
    
    addr = addr.strip().rstrip(', ')
    if 'córdoba' not in addr.lower():
        addr += ', Córdoba, España'
    return addr


def _smart_geocode(addr: str, nomi_rate, nomi_raw, photon, timeout=8):
    q = _normalize(addr)
    to_loc = lambda g: Location(q, g.latitude, g.longitude)

    if (g := nomi_rate(q, timeout=timeout)):
        return to_loc(g)

    try:
        cand = nomi_raw.geocode(q, exactly_one=False, limit=5, timeout=timeout)
    except Exception:
        cand = None
    if cand:
        best = max(cand, key=lambda c: fuzz.token_set_ratio(q.lower(), c.address.lower()))
        if fuzz.token_set_ratio(q.lower(), best.address.lower()) > 60:
            return to_loc(best)

    try:
        if (g := photon.geocode(q, timeout=timeout)):
            return to_loc(g)
    except Exception:
        pass
    return None

# ──────────────────────────────────────────────────────────────────────────────
# Construir matriz de distancias con OSRM público
# ──────────────────────────────────────────────────────────────────────────────

def _build_matrix(locs: Sequence[Location]):
    coords = ';'.join(f'{l.lon},{l.lat}' for l in locs)
    url = f'https://router.project-osrm.org/table/v1/driving/{coords}?annotations=distance'
    r = requests.get(url, timeout=60)
    base = [[int(c) if c is not None else 10**9 for c in row] for row in r.json()['distances']]

    # nodo dummy para rutas abiertas
    n = len(base)
    for row in base:
        row.append(0)
    base.append([0] * (n + 1))
    return base, n

# ──────────────────────────────────────────────────────────────────────────────
# Solucionador VRP con OR‑Tools
# ──────────────────────────────────────────────────────────────────────────────

def _solve_vrp(dist, k: int, dummy: int):
    man = pywrapcp.RoutingIndexManager(len(dist), k, [0]*k, [dummy]*k)
    model = pywrapcp.RoutingModel(man)

    transit = model.RegisterTransitCallback(lambda i,j: dist[man.IndexToNode(i)][man.IndexToNode(j)])
    model.SetArcCostEvaluatorOfAllVehicles(transit)

    demand = model.RegisterUnaryTransitCallback(
        lambda idx: 0 if man.IndexToNode(idx) in (0, dummy) else 1)
    max_load = math.ceil((len(dist) - 2) / k)     # -2: depósito + dummy
    model.AddDimensionWithVehicleCapacity(
        demand, 0, [max_load]*k, True, 'Load')
    
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.time_limit.seconds = 10
    sol = model.SolveWithParameters(params)
    if not sol:
        raise RuntimeError('No se pudo hallar solución')

    routes = []
    for v in range(k):
        idx = model.Start(v)
        seq = []
        while not model.IsEnd(idx):
            node = man.IndexToNode(idx)
            if node not in (0, dummy):
                seq.append(node)
            idx = sol.Value(model.NextVar(idx))
        routes.append(seq)
    return routes

# ──────────────────────────────────────────────────────────────────────────────
# Función principal de línea de comandos
# ──────────────────────────────────────────────────────────────────────────────

#@click.command()
#@click.option('-t', '--trucks', default=2, type=int, required=True, help='Número de camiones')
#@click.option('-f', '--file', 'file_', default='direcciones.txt', type=click.Path(exists=True), required=True, help='Fichero .txt con direcciones')
#@click.option('--json-out', is_flag=True, help='Exportar rutas a JSON')

def opt(trucks, direcciones):

    #addresses = [l.strip() for l in Path(file_).read_text(encoding='utf-8').splitlines() if l.strip()]
    addresses = direcciones
    nomi_raw = Nominatim(user_agent='route_opt_cli')
    nomi_rate = RateLimiter(nomi_raw.geocode, min_delay_seconds=1, max_retries=2)
    photon = Photon(user_agent='route_opt_cli')

    depot = 'Corte Inglés Ronda Poniente, Córdoba, España'
    depot_loc = _smart_geocode(depot, nomi_rate, nomi_raw, photon)
    if depot_loc is None:
        console.print('[red]Depósito no geocodificado'); return

    locs = [depot_loc]
    unresolved = []
    for a in addresses:
        loc = _smart_geocode(a, nomi_rate, nomi_raw, photon)
        if loc:
            locs.append(loc)
        else:
            unresolved.append(a)

    if not locs[1:]:
        console.print('[red]Ninguna dirección válida'); return

    dist, dummy = _build_matrix(locs)
    idx_routes = _solve_vrp(dist, trucks, dummy)
    routes = [[locs[0].address] + [locs[i].address for i in seq] for seq in idx_routes]
    # Tabla
    tbl = Table(show_header=True, header_style='bold cyan')
    tbl.add_column('Camión'); tbl.add_column('Ruta')
    for i, r in enumerate(routes, 1):
        tbl.add_row(str(i), ' ➜ '.join(r))
    #console.print(tbl)

    if unresolved:
        console.print(f'[yellow]Sin geocodificar ({len(unresolved)}):[/]\n- ' + '\n- '.join(unresolved))

    return routes
if __name__ == '__main__':
    opt(2, 'direcciones.txt')
