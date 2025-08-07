#!/usr/bin/env python3
"""
Debug mode runner - Start only the web server for robot state testing
This allows you to test robot state management without running actual robots or fleet manager.
"""

import os
import sys

# Add web directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'web'))

from web import create_app

if __name__ == "__main__":
    print("🔧 Starting Robot Debug Mode...")
    print("📍 Access the debug tool at: http://localhost:8080/robot_debug")
    print("🌐 Main dashboard at: http://localhost:8080/")
    print("\nThis mode allows you to:")
    print("  • Manually control robot states")
    print("  • Create test tasks")
    print("  • Simulate delivery/call scenarios")
    print("  • Test without actual robots or fleet manager")
    print("\n" + "="*50)
    
    app = create_app()
    
    # Configure Flask for debug mode
    app.config['DEBUG'] = True
    app.secret_key = 'debug_secret_key_for_testing'
    
    try:
        app.run(host='0.0.0.0', port=8080, debug=True)
    except KeyboardInterrupt:
        print("\n🛑 Debug mode stopped")
    except Exception as e:
        print(f"❌ Error starting debug mode: {e}")
        sys.exit(1)