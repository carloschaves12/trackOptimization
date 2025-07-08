# app.py – servidor Flask minimal que sirve la página y llama al motor
from flask import Flask, request, jsonify, render_template
from optimization import compute_routes

app = Flask(__name__, static_folder='static', template_folder='templates')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/optimize', methods=['POST'])
def optimize():
    data = request.get_json(force=True)
    addresses = data.get('addresses', [])
    if isinstance(addresses, str):
        addresses = [s.strip() for s in addresses.splitlines() if s.strip()]
    trucks = data.get('trucks')

    try:
        trucks = int(trucks)
        routes = compute_routes(addresses, trucks)
    except Exception as e:
        return jsonify({'error': str(e)}), 400

    return jsonify({'routes': routes})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
