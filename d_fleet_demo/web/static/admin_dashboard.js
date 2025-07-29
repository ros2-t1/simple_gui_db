document.addEventListener('DOMContentLoaded', () => {
    let updateInterval = null;
    let charts = {};
    let currentData = null;

    // Chart.js configuration
    Chart.defaults.color = '#cccccc';
    Chart.defaults.borderColor = '#333333';
    Chart.defaults.backgroundColor = 'rgba(0, 170, 255, 0.1)';

    // Theme colors
    const colors = {
        primary: '#00aaff',
        secondary: '#ff6b6b', 
        success: '#51cf66',
        warning: '#ffd43b',
        danger: '#ff4444',
        info: '#00aaff',
        muted: '#888888'
    };

    // Utility functions
    function showNotification(message, type = 'info', duration = 5000) {
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: ${type === 'error' ? colors.danger : type === 'success' ? colors.success : colors.info};
            color: white;
            padding: 15px 20px;
            border-radius: 8px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
            z-index: 9999;
            font-size: 14px;
            max-width: 300px;
            animation: slideIn 0.3s ease-out;
        `;
        
        notification.innerHTML = `
            <div style="display: flex; align-items: center; gap: 10px;">
                <i class="bi bi-${type === 'error' ? 'exclamation-triangle' : type === 'success' ? 'check-circle' : 'info-circle'}"></i>
                <span>${message}</span>
            </div>
        `;
        
        document.body.appendChild(notification);
        
        setTimeout(() => {
            notification.remove();
        }, duration);
    }

    function formatDateTime(dateString) {
        if (!dateString) return 'N/A';
        const date = new Date(dateString);
        return date.toLocaleString('ko-KR', {
            year: 'numeric',
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            hour12: false
        });
    }

    function getStatusBadgeHtml(status) {
        const statusMap = {
            'idle': 'IDLE', 'busy': 'BUSY', 'error': 'ERROR', 'charging': 'CHARGING',
            '대기': 'PENDING', '할당': 'ASSIGNED', '이동중': 'IN_PROGRESS', '집기중': 'PICKING',
            '수령대기': 'WAITING', '완료': 'COMPLETED', '실패': 'FAILED'
        };
        const statusClassMap = {
            'idle': 'idle', 'busy': 'busy', 'error': 'error', 'charging': 'charging',
            '대기': 'pending', '할당': 'assigned', '이동중': 'in_progress', '집기중': 'in_progress',
            '수령대기': 'in_progress', '완료': 'completed', '실패': 'failed'
        };
        const displayStatus = statusMap[status] || status.toUpperCase();
        const statusClass = statusClassMap[status] || 'muted';
        return `<span class="status-badge ${statusClass}">${displayStatus}</span>`;
    }

    // Section switching logic
    const sidebarLinks = document.querySelectorAll('.sidebar-nav .nav-link');
    const sections = document.querySelectorAll('.section-content');
    const currentSectionTitle = document.getElementById('current-section-title');
    const currentSectionSubtitle = document.getElementById('current-section-subtitle');

    function showSection(sectionId) {
        sections.forEach(section => {
            section.classList.remove('active');
        });
        document.getElementById(`${sectionId}-section`).classList.add('active');

        sidebarLinks.forEach(link => {
            link.classList.remove('active');
            if (link.dataset.section === sectionId) {
                link.classList.add('active');
            }
        });

        // Update header title and subtitle
        const sectionTitles = {
            'dashboard': { title: 'Dashboard', subtitle: 'Overview of fleet operations' },
            'robots': { title: 'Robot Management', subtitle: 'Monitor and control individual robots' },
            'tasks': { title: 'Task Management', subtitle: 'View and manage all tasks' },
            'users': { title: 'User Management', subtitle: 'Manage registered users' },
            'items': { title: 'Item Management', subtitle: 'View and update item inventory' }
        };
        currentSectionTitle.textContent = sectionTitles[sectionId].title;
        currentSectionSubtitle.textContent = sectionTitles[sectionId].subtitle;
    }

    sidebarLinks.forEach(link => {
        link.addEventListener('click', (e) => {
            e.preventDefault();
            showSection(e.target.dataset.section);
            refreshData(); // Refresh data when switching sections
        });
    });

    // Initialize charts (from advanced_fleet_dashboard.js)
    function initCharts() {
        const hourlyCtx = document.getElementById('hourlyChart').getContext('2d');
        charts.hourly = new Chart(hourlyCtx, {
            type: 'line',
            data: {
                labels: Array.from({length: 24}, (_, i) => `${i}:00`),
                datasets: [{ label: 'Total Tasks', data: [], borderColor: colors.primary, backgroundColor: colors.primary + '20', fill: true, tension: 0.4 }, { label: 'Completed', data: [], borderColor: colors.success, backgroundColor: colors.success + '20', fill: true, tension: 0.4 }, { label: 'Failed', data: [], borderColor: colors.danger, backgroundColor: colors.danger + '20', fill: true, tension: 0.4 }]
            },
            options: { responsive: true, maintainAspectRatio: false, scales: { y: { beginAtZero: true, grid: { color: '#333333' } }, x: { grid: { color: '#333333' } } }, plugins: { legend: { display: true, position: 'top' } } }
        });

        const statusCtx = document.getElementById('statusChart').getContext('2d');
        charts.status = new Chart(statusCtx, {
            type: 'doughnut',
            data: {
                labels: ['Completed', 'Failed', 'Active', 'Pending'],
                datasets: [{ data: [0, 0, 0, 0], backgroundColor: [colors.success, colors.danger, colors.warning, colors.muted], borderWidth: 2, borderColor: '#1e1e1e' }]
            },
            options: { responsive: true, maintainAspectRatio: false, plugins: { legend: { position: 'bottom' } } }
        });

        const robotCtx = document.getElementById('robotChart').getContext('2d');
        charts.robot = new Chart(robotCtx, {
            type: 'bar',
            data: {
                labels: [],
                datasets: [{ label: 'Completed Tasks', data: [], backgroundColor: colors.success + '80', borderColor: colors.success, borderWidth: 1 }, { label: 'Failed Tasks', data: [], backgroundColor: colors.danger + '80', borderColor: colors.danger, borderWidth: 1 }]
            },
            options: { responsive: true, maintainAspectRatio: false, scales: { y: { beginAtZero: true, grid: { color: '#333333' } }, x: { grid: { color: '#333333' } } } }
        });

        const trendCtx = document.getElementById('trendChart').getContext('2d');
        charts.trend = new Chart(trendCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{ label: 'Daily Tasks', data: [], borderColor: colors.primary, backgroundColor: colors.primary + '20', fill: true, tension: 0.4 }, { label: 'Success Rate %', data: [], borderColor: colors.success, backgroundColor: colors.success + '20', fill: false, yAxisID: 'percentage', tension: 0.4 }]
            },
            options: { responsive: true, maintainAspectRatio: false, scales: { y: { beginAtZero: true, grid: { color: '#333333' } }, percentage: { type: 'linear', display: true, position: 'right', min: 0, max: 100, grid: { drawOnChartArea: false, }, }, x: { grid: { color: '#333333' } } } }
        });
    }

    // Update metrics display (from advanced_fleet_dashboard.js)
    function updateMetrics(metrics) {
        document.getElementById('total-tasks-today').textContent = metrics.total_tasks_today || 0;
        const avgTime = metrics.avg_completion_time;
        document.getElementById('avg-completion-time').textContent = (avgTime && typeof avgTime === 'number') ? avgTime.toFixed(1) : '0.0';
        document.getElementById('active-users').textContent = metrics.unique_users_today || 0;
        document.getElementById('active-robots').textContent = metrics.active_robots_today || 0;
        const successRate = (metrics.total_tasks_today && metrics.total_tasks_today > 0) ? ((metrics.completed_today / metrics.total_tasks_today) * 100).toFixed(1) : '0.0';
        document.getElementById('success-rate').textContent = successRate + '%';
        document.getElementById('failed-tasks').textContent = metrics.failed_today || 0;

        updateChangeIndicator('tasks-change', 12, 'positive');
        updateChangeIndicator('time-change', -5, 'positive');
        updateChangeIndicator('users-change', 3, 'positive');
        updateChangeIndicator('success-change', 2.3, 'positive');
    }

    function updateChangeIndicator(elementId, change, type) {
        const element = document.getElementById(elementId);
        const prefix = change > 0 ? '+' : '';
        const suffix = elementId.includes('change') && !elementId.includes('users') ? '%' : '';
        element.textContent = `${prefix}${change}${suffix}`;
        element.className = `metric-change ${type}`;
    }

    function updateHourlyChart(hourlyData) {
        const hours = Array.from({length: 24}, (_, i) => i);
        const taskCounts = new Array(24).fill(0);
        const completedCounts = new Array(24).fill(0);
        const failedCounts = new Array(24).fill(0);

        hourlyData.forEach(item => {
            const hour = parseInt(item.hour);
            taskCounts[hour] = item.task_count || 0;
            completedCounts[hour] = item.completed_count || 0;
            failedCounts[hour] = item.failed_count || 0;
        });

        charts.hourly.data.datasets[0].data = taskCounts;
        charts.hourly.data.datasets[1].data = completedCounts;
        charts.hourly.data.datasets[2].data = failedCounts;
        charts.hourly.update();
    }

    function updateStatusChart(metrics) {
        const data = [
            metrics.completed_today || 0,
            metrics.failed_today || 0,
            metrics.active_today || 0,
            (metrics.total_tasks_today || 0) - (metrics.completed_today || 0) - (metrics.failed_today || 0) - (metrics.active_today || 0)
        ];
        charts.status.data.datasets[0].data = data;
        charts.status.update();
    }

    function updateRobotChart(robotData) {
        const names = robotData.map(robot => robot.bot_name || `Robot ${robot.hana_bot_id}`);
        const completed = robotData.map(robot => robot.completed_tasks || 0);
        const failed = robotData.map(robot => robot.failed_tasks || 0);

        charts.robot.data.labels = names;
        charts.robot.data.datasets[0].data = completed;
        charts.robot.data.datasets[1].data = failed;
        charts.robot.update();
    }

    function updateTrendChart(trendData) {
        const dates = trendData.map(item => {
            const date = new Date(item.date);
            return `${date.getMonth() + 1}/${date.getDate()}`;
        }).reverse();
        
        const tasks = trendData.map(item => item.total_tasks || 0).reverse();
        const successRates = trendData.map(item => {
            const total = item.total_tasks || 0;
            const completed = item.completed_tasks || 0;
            return total > 0 ? parseFloat(((completed / total) * 100).toFixed(1)) : 0;
        }).reverse();

        charts.trend.data.labels = dates;
        charts.trend.data.datasets[0].data = tasks;
        charts.trend.data.datasets[1].data = successRates;
        charts.trend.update();
    }

    function updateTopUsersTable(userData) {
        const tbody = document.getElementById('top-users-table');
        if (!userData || userData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="5" class="text-center text-muted">No data available</td></tr>';
            return;
        }
        let html = '';
        userData.forEach((user) => {
            const successRate = user.order_count > 0 ? ((user.completed_orders / user.order_count) * 100).toFixed(1) : '0.0';
            const performanceClass = successRate >= 90 ? 'excellent' : successRate >= 70 ? 'good' : 'poor';
            html += `
                <tr>
                    <td><strong>${user.name}</strong><br><small class="text-muted">ID: ${user.resident_id}</small></td>
                    <td><strong>${user.order_count}</strong></td>
                    <td>${user.completed_orders}</td>
                    <td>${successRate}%</td>
                    <td><div class="performance-bar"><div class="performance-fill ${performanceClass}" style="width: ${successRate}%"></div></div></td>
                </tr>
            `;
        });
        tbody.innerHTML = html;
    }

    function updatePopularItemsTable(itemData) {
        const tbody = document.getElementById('popular-items-table');
        if (!itemData || itemData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="5" class="text-center text-muted">No data available</td></tr>';
            return;
        }
        let html = '';
        const maxOrders = Math.max(...itemData.map(item => item.order_count));
        itemData.forEach((item) => {
            const successRate = item.order_count > 0 ? ((item.delivered_count / item.order_count) * 100).toFixed(1) : '0.0';
            const popularity = maxOrders > 0 ? ((item.order_count / maxOrders) * 100).toFixed(0) : 0;
            html += `
                <tr>
                    <td><strong>${item.item_type}</strong><br><small class="text-muted">ID: ${item.item_id}</small></td>
                    <td><strong>${item.order_count}</strong></td>
                    <td>${item.delivered_count}</td>
                    <td>${successRate}%</td>
                    <td><div class="performance-bar"><div class="performance-fill excellent" style="width: ${popularity}%"></div></div></td>
                </tr>
            `;
        });
        tbody.innerHTML = html;
    }

    // Fetch analytics data (from advanced_fleet_dashboard.js)
    async function fetchAnalytics() {
        try {
            const response = await fetch('/api/fleet/analytics');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                currentData = result.data;
                updateMetrics(currentData.system_metrics);
                updateHourlyChart(currentData.hourly_distribution);
                updateStatusChart(currentData.system_metrics);
                updateRobotChart(currentData.robot_performance);
                updateTrendChart(currentData.daily_trends);
                updateTopUsersTable(currentData.top_requesters);
                updatePopularItemsTable(currentData.popular_items);
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch analytics:', error);
            showNotification('Failed to load analytics data: ' + error.message, 'error');
        }
    }

    // Control functions (from advanced_fleet_dashboard.js)
    async function sendControlCommand(action, targetId = null) {
        try {
            const response = await fetch('/api/fleet/control', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action: action, target_id: targetId })
            });
            const result = await response.json();
            if (result.success) {
                showNotification(result.message, 'success');
                setTimeout(refreshData, 1000);
            } else {
                showNotification('Control action failed: ' + result.error, 'error');
            }
        } catch (error) {
            console.error('Control command failed:', error);
            showNotification('Control command failed: ' + error.message, 'error');
        }
    }

    // New data fetching and rendering for admin sections
    async function fetchRobots() {
        try {
            const response = await fetch('/api/admin/robots');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('robots-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="7" class="text-center text-muted">No robot data available</td></tr>';
                } else {
                    result.data.forEach(robot => {
                        html += `
                            <tr>
                                <td>${robot.id}</td>
                                <td>${robot.name}</td>
                                <td>${getStatusBadgeHtml(robot.status)}</td>
                                <td>${robot.battery || 'N/A'}%</td>
                                <td>${robot.current_task ? `#${robot.current_task.task_id} (${robot.current_task.task_type})` : 'None'}</td>
                                <td>${formatDateTime(robot.last_update)}</td>
                                <td>
                                    <button class="action-icon-btn" onclick="resetRobot('${robot.id}')" title="Reset Robot"><i class="bi bi-arrow-clockwise"></i></button>
                                    <button class="action-icon-btn danger" onclick="emergencyStopRobot('${robot.id}')" title="Emergency Stop"><i class="bi bi-stop-circle"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch robots:', error);
            showNotification('Failed to load robot data: ' + error.message, 'error');
        }
    }

    async function fetchTasks() {
        try {
            const response = await fetch('/api/admin/tasks');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('tasks-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="9" class="text-center text-muted">No task data available</td></tr>';
                } else {
                    result.data.forEach(task => {
                        html += `
                            <tr>
                                <td>${task.task_id}</td>
                                <td>${task.task_type}</td>
                                <td>${getStatusBadgeHtml(task.status)}</td>
                                <td>${task.requester_name || 'N/A'}</td>
                                <td>${task.item_type || 'N/A'}</td>
                                <td>${task.bot_name || 'Unassigned'}</td>
                                <td>${formatDateTime(task.created_at)}</td>
                                <td>${formatDateTime(task.completed_at)}</td>
                                <td>
                                    <button class="action-icon-btn danger" onclick="cancelTask(${task.task_id})" title="Cancel Task"><i class="bi bi-x-circle"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch tasks:', error);
            showNotification('Failed to load task data: ' + error.message, 'error');
        }
    }

    async function fetchUsers() {
        try {
            const response = await fetch('/api/admin/users');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('users-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="6" class="text-center text-muted">No user data available</td></tr>';
                } else {
                    result.data.forEach(user => {
                        html += `
                            <tr>
                                <td>${user.resident_id}</td>
                                <td>${user.name}</td>
                                <td>${user.gender}</td>
                                <td>${user.birth_date}</td>
                                <td>${user.login_id}</td>
                                <td>
                                    <button class="action-icon-btn" onclick="editUser(${user.resident_id})" title="Edit User"><i class="bi bi-pencil"></i></button>
                                    <button class="action-icon-btn danger" onclick="deleteUser(${user.resident_id})" title="Delete User"><i class="bi bi-trash"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch users:', error);
            showNotification('Failed to load user data: ' + error.message, 'error');
        }
    }

    async function fetchItems() {
        try {
            const response = await fetch('/api/admin/items');
            if (!response.ok) throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            const result = await response.json();
            if (result.success) {
                const tbody = document.getElementById('items-table-body');
                let html = '';
                if (result.data.length === 0) {
                    html = '<tr><td colspan="4" class="text-center text-muted">No item data available</td></tr>';
                } else {
                    result.data.forEach(item => {
                        html += `
                            <tr>
                                <td>${item.item_id}</td>
                                <td>${item.item_type}</td>
                                <td>${item.item_quantity}</td>
                                <td>
                                    <button class="action-icon-btn" onclick="editItem(${item.item_id})" title="Edit Item"><i class="bi bi-pencil"></i></button>
                                </td>
                            </tr>
                        `;
                    });
                }
                tbody.innerHTML = html;
            } else {
                throw new Error(result.error || 'Unknown API error');
            }
        } catch (error) {
            console.error('Failed to fetch items:', error);
            showNotification('Failed to load item data: ' + error.message, 'error');
        }
    }

    // Global refresh function
    window.refreshData = () => {
        showNotification('Refreshing data...', 'info');
        fetchAnalytics();
        fetchRobots();
        fetchTasks();
        fetchUsers();
        fetchItems();
    };

    // Global control functions (placeholders for now)
    window.emergencyStop = () => { if (confirm('Are you sure you want to issue an emergency stop to all robots?')) sendControlCommand('emergency_stop'); };
    window.resetSystem = () => { if (confirm('Reset all robot statuses to idle?')) sendControlCommand('reset_system'); };
    window.optimizeRoutes = () => { showNotification('Route optimization initiated...', 'info'); sendControlCommand('optimize_routes'); };
    window.maintenanceMode = () => { if (confirm('Enable maintenance mode? This will pause all operations.')) sendControlCommand('maintenance_mode'); };
    window.generateReport = () => { showNotification('Generating comprehensive fleet report...', 'info'); setTimeout(() => { showNotification('Report generation completed', 'success'); }, 3000); };
    window.clearFailedTasks = () => { if (confirm('Clear all failed tasks from the database?')) sendControlCommand('fail_all_incomplete'); };

    // Specific robot/task/user/item actions (placeholders)
    window.resetRobot = (robotId) => { showNotification(`Resetting robot ${robotId}...`, 'info'); /* Implement API call */ };
    window.emergencyStopRobot = (robotId) => { if (confirm(`Emergency stop robot ${robotId}?`)) showNotification(`Emergency stopping robot ${robotId}...`, 'warning'); /* Implement API call */ };
    window.cancelTask = (taskId) => { if (confirm(`Cancel task ${taskId}?`)) sendControlCommand('cancel_task', taskId); };
    window.editUser = (userId) => { showNotification(`Editing user ${userId}...`, 'info'); /* Implement UI/API for editing */ };
    window.deleteUser = (userId) => { if (confirm(`Delete user ${userId}?`)) showNotification(`Deleting user ${userId}...`, 'warning'); /* Implement API call */ };
    window.editItem = (itemId) => { showNotification(`Editing item ${itemId}...`, 'info'); /* Implement UI/API for editing */ };

    // Initial dashboard setup
    async function initDashboard() {
        console.log('HANA Admin Dashboard initialized');
        initCharts();
        showSection('dashboard'); // Start with dashboard section
        refreshData();
        updateInterval = setInterval(refreshData, 10000); // Auto-refresh every 10 seconds
    }

    window.addEventListener('beforeunload', () => {
        if (updateInterval) clearInterval(updateInterval);
    });

    document.addEventListener('click', (e) => {
        if (e.target.classList.contains('chart-control')) {
            e.target.parentElement.querySelectorAll('.chart-control').forEach(btn => {
                btn.classList.remove('active');
            });
            e.target.classList.add('active');
            const period = e.target.dataset.period;
            showNotification(`Switching to ${period} view...`, 'info');
        }
    });

    initDashboard();
});