from flask import Flask
from .routes import auth, items, orders, status, fleet_monitor

def create_app():
    app = Flask(__name__, template_folder="templates", static_folder="static")
    app.register_blueprint(auth.bp)
    app.register_blueprint(items.bp)
    app.register_blueprint(orders.bp)
    app.register_blueprint(status.bp)
    app.register_blueprint(fleet_monitor.bp)
    return app
