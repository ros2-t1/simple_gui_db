from flask import Flask
from .routes import auth, items, orders

def create_app():
    app = Flask(__name__, template_folder="templates", static_folder="static")
    app.register_blueprint(auth.bp)
    app.register_blueprint(items.bp)
    app.register_blueprint(orders.bp)
    return app
