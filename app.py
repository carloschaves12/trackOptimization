from flask import Flask, render_template, request, redirect, url_for, flash
import optimization

app = Flask(__name__)
app.secret_key = "cambia_esto_por_un_valor_secreto"

# LISTA en memoria para la demo (reinicia al cerrar la app)
rutas = []

@app.get("/")
def index():
    return render_template("index.html", rutas=rutas)

@app.post("/agregar")
def agregar():
    camion = request.form.get("camion", type=int, default=2)
    direcciones = request.form.get("direcciones", "").strip().splitlines()
    n = optimization.opt(camion, direcciones)
    rutas.extend(n)
    return redirect(url_for("index"))

@app.post("/limpiar")
def limpiar():
    rutas.clear()                    # vacía la lista
    flash("Rutas eliminadas", "success")
    return redirect(url_for("index"))

if __name__ == "__main__":
    app.run(debug=True)
