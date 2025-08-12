from flask import Flask
from .routes import auth, items, orders, status, fleet_monitor, order2, camera
from .routes.status import initialize_robot_status_from_db

def create_app():
    app = Flask(__name__, template_folder="templates", static_folder="static")
    app.register_blueprint(auth.bp)
    app.register_blueprint(items.bp)
    app.register_blueprint(orders.bp)
    app.register_blueprint(status.bp)
    app.register_blueprint(fleet_monitor.bp)
    app.register_blueprint(order2.order2_bp)
    app.register_blueprint(camera.bp)
    
    # Initialize robot status cache from database on startup
    with app.app_context():
        initialize_robot_status_from_db()
    
    return app
