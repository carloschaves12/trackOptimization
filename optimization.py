#!/usr/bin/env python3
"""route_optimizer.py (v3.1 – sintaxis corregida)

* `SyntaxError: '{' was never closed` solucionado (bloque JSON de salida).
* Rutas abiertas por defecto (`--closed-route` para volver al depósito).
"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Sequence

import click
import requests
from geopy import Nominatim
from geopy.extra.rate_limiter import RateLimiter
from geopy.geocoders import Photon
from rapidfuzz import fuzz
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from rich import print
from rich.console import Console
from rich.table import Table

console = Console()

###############################################################################
# Dataclass -------------------------------------------------------------------
###############################################################################

@dataclass
class Location:
    address: str
    lat: float
    lon: float

###############################################################################
# Geocoding utilities ---------------------------------------------------------
###############################################################################

def _normalize(addr: str) -> str:
    addr = addr.strip().rstrip(", ")
    # Replace common abbreviations
    reps = {"av.": "avenida", "c.": "calle", "nº": "", " s/n": ""}
    for k, v in reps.items():
        addr = addr.replace(k, v).replace(k.capitalize(), v)
    if "córdoba" not in addr.lower():
        addr += ", Córdoba, España"
    return addr


def smart_geocode(addr: str, nomi_rate: RateLimiter, nomi_raw: Nominatim, photon: Photon, timeout: int):
    query = _normalize(addr)

    def to_loc(g):
        return Location(query, g.latitude, g.longitude)

    g = nomi_rate(query, timeout=timeout)
    if g:
        return to_loc(g)

    bbox = "-4.85,38.0,-4.60,37.80"  # Córdoba bounding box
    try:
        cands = nomi_raw.geocode(query, exactly_one=False, limit=5, viewbox=bbox, bounded=True, timeout=timeout)
    except Exception:
        cands = None
    if cands:
        best = max(cands, key=lambda c: fuzz.token_set_ratio(query.lower(), c.address.lower()))
        if fuzz.token_set_ratio(query.lower(), best.address.lower()) > 60:
            return to_loc(best)

    try:
        g = photon.geocode(query, timeout=timeout)
    except Exception:
        g = None
    if g:
        return to_loc(g)
    return None

###############################################################################
# Distance matrix -------------------------------------------------------------
###############################################################################

def build_distance_matrix(locs: Sequence[Location], osrm_url: str, open_route: bool):
    coords = ";".join(f"{l.lon},{l.lat}" for l in locs)
    r = requests.get(f"{osrm_url.rstrip('/')}/table/v1/driving/{coords}?annotations=distance", timeout=60)
    base = [[int(c) if c is not None else 10**9 for c in row] for row in r.json().get("distances", [])]

    if not open_route:
        return base, None

    n = len(base)
    for row in base:
        row.append(0)
    base.append([0] * (n + 1))
    return base, n  # dummy index = n

###############################################################################
# VRP solver ------------------------------------------------------------------
###############################################################################

def solve_vrp(dist, k: int, balance: bool, span: bool, n_stops: int, dummy: int | None):
    if dummy is None:
        man = pywrapcp.RoutingIndexManager(len(dist), k, 0)
    else:
        man = pywrapcp.RoutingIndexManager(len(dist), k, [0] * k, [dummy] * k)
    model = pywrapcp.RoutingModel(man)

    transit = model.RegisterTransitCallback(lambda i, j: dist[man.IndexToNode(i)][man.IndexToNode(j)])
    model.SetArcCostEvaluatorOfAllVehicles(transit)

    if balance:
        demand = model.RegisterUnaryTransitCallback(lambda idx: 0 if man.IndexToNode(idx) in (0, dummy) else 1)
        model.AddDimensionWithVehicleCapacity(demand, 0, [math.ceil(n_stops / k)] * k, True, "Load")

    if span:
        model.AddDimension(transit, 0, 10**9, True, "Span")
        model.GetDimensionOrDie("Span").SetGlobalSpanCostCoefficient(100)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = 15

    sol = model.SolveWithParameters(params)
    if not sol:
        console.print("[red]Sin solución VRP.")
        sys.exit(1)

    routes = []
    for v in range(k):
        idx = model.Start(v)
        route = []
        while not model.IsEnd(idx):
            node = man.IndexToNode(idx)
            if node != dummy:
                route.append(node)
            idx = sol.Value(model.NextVar(idx))
        routes.append(route)
    return routes

###############################################################################
# CLI -------------------------------------------------------------------------
###############################################################################

@click.command()
@click.option('--trucks', '-t', type=int, required=True)
@click.option('--address', '-a', multiple=True)
@click.option('--file', '-f', type=click.Path(exists=True))
@click.option('--depot', default='Corte Inglés Ronda Poniente')
@click.option('--osrm-url', default='https://router.project-osrm.org')
@click.option('--timeout', default=8)
@click.option('--balance', is_flag=True)
@click.option('--span', is_flag=True)
@click.option('--open-route/--closed-route', default=True)
@click.option('--no-interactive', is_flag=True)
@click.option('--json-out', is_flag=True)

def cli(trucks, address, file, depot, osrm_url, timeout, balance, span, open_route, no_interactive, json_out):
    # gather addresses
    addrs = list(address)
    if file:
        addrs.extend(Path(file).read_text(encoding='utf-8').splitlines())
    addrs = [a.strip() for a in addrs if a.strip() and not a.strip().startswith('#')]
    if not addrs:
        console.print('[red]Sin direcciones.'); sys.exit(1)

    routes = compute_routes(addrs, trucks,
                        depot=depot,
                        osrm_url=osrm_url,
                        timeout=timeout,
                        balance=balance,
                        span=span,
                        open_route=open_route,
                        interactive=not no_interactive)
    
    return routes


# —— NUEVA FUNCIÓN REUTILIZABLE ——————————————————————
def compute_routes(addresses: list[str],
                   trucks: int,
                   depot: str = 'Corte Inglés Ronda Poniente',
                   osrm_url: str = 'https://router.project-osrm.org',
                   timeout: int = 8,
                   balance: bool = True,
                   span: bool = True,
                   open_route: bool = True,
                   interactive: bool = False,
                   json_out: bool = True):

    # geocoders
    nomi_raw = Nominatim(user_agent='route_opt_cli')
    nomi_rate = RateLimiter(nomi_raw.geocode, min_delay_seconds=1, max_retries=2, error_wait_seconds=2)
    photon = Photon(user_agent='route_opt_cli')

    def resolve(a):
        loc = smart_geocode(a, nomi_rate, nomi_raw, photon, timeout)
        while loc is None and not interactive:
            console.print(f'[yellow]No encontrada:[/] {a}')
            new = console.input('📝 Corrige o Enter para omitir > ').strip()
            if not new:
                return None
            loc = smart_geocode(new, nomi_rate, nomi_raw, photon, timeout)
        return loc

    console.print(f'[bold]Geocodificando {len(addresses) + 1} direcciones…[/]')
    depot_loc = resolve(depot)
    valid_addrs = []
    stops = []
    for addr in addresses:
        loc = resolve(addr)
        if loc:
            valid_addrs.append(addr)
            stops.append(loc)

    if not depot_loc or not stops:
        console.print('[red]Geocodificación insuficiente.'); sys.exit(1)

    locs = [depot_loc] + stops
    console.print('[bold]Construyendo matriz de distancias…[/]')
    dist, dummy = build_distance_matrix(locs, osrm_url, open_route)

    console.print('[bold]Resolviendo VRP…[/]')
    idx_routes = solve_vrp(dist, trucks, balance, span, len(stops), dummy)
    routes = [[valid_addrs[i - 1] for i in route if i != 0] for route in idx_routes]
    return routes


if __name__ == '__main__':
    cli()
