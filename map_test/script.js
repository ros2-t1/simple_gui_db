const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');

// --- Configuration ---
const cellSize = 40; // Size of each grid cell in pixels
const obstacleColor = '#2c3e50';
const freeSpaceColor = '#34495e';
const gridLineColor = '#4a627a';
const robotColor = '#1abc9c';
const robotNeonColor = 'rgba(26, 188, 156, 0.7)';
const robotSize = cellSize * 0.7; // Robot size relative to cell size
const animationSpeed = 0.06; // Speed of robot movement interpolation

// --- Map and Robot State ---
const grid = mapData.grid;
const path = mapData.path;
const mapWidth = grid[0].length;
const mapHeight = grid.length;

let pathIndex = 0;
let robot = {
    x: path[0].x,
    y: path[0].y,
    angle: 0,
    targetX: path[0].x,
    targetY: path[0].y,
    neonPulse: 0, // For the glowing effect
    neonDirection: 1
};

// --- Canvas Setup ---
canvas.width = mapWidth * cellSize;
canvas.height = mapHeight * cellSize;

// --- Drawing Functions ---

function drawGrid() {
    for (let y = 0; y < mapHeight; y++) {
        for (let x = 0; x < mapWidth; x++) {
            ctx.fillStyle = grid[y][x] === 1 ? obstacleColor : freeSpaceColor;
            ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
            ctx.strokeStyle = gridLineColor;
            ctx.strokeRect(x * cellSize, y * cellSize, cellSize, cellSize);
        }
    }
}

function drawRobot() {
    const canvasX = robot.x * cellSize + cellSize / 2;
    const canvasY = robot.y * cellSize + cellSize / 2;

    ctx.save();
    ctx.translate(canvasX, canvasY);

    // Draw neon glow
    const glowSize = robotSize * (1.5 + robot.neonPulse * 0.5);
    ctx.shadowBlur = 20;
    ctx.shadowColor = robotNeonColor;
    ctx.fillStyle = robotNeonColor;
    ctx.beginPath();
    ctx.arc(0, 0, glowSize / 2, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0; // Reset shadow for other elements

    // Rotate for the pointer
    ctx.rotate(robot.angle);

    // Draw robot body (circle)
    ctx.fillStyle = robotColor;
    ctx.beginPath();
    ctx.arc(0, 0, robotSize / 2, 0, 2 * Math.PI);
    ctx.fill();

    // Draw direction pointer
    ctx.fillStyle = '#ecf0f1';
    ctx.beginPath();
    ctx.moveTo(0, -robotSize * 0.4);
    ctx.lineTo(-robotSize * 0.15, -robotSize * 0.1);
    ctx.lineTo(robotSize * 0.15, -robotSize * 0.1);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

// --- Animation Logic ---

function updateRobot() {
    // Animate neon pulse
    robot.neonPulse += 0.05 * robot.neonDirection;
    if (robot.neonPulse > 1 || robot.neonPulse < 0) {
        robot.neonDirection *= -1;
    }

    // Interpolate position for smooth movement
    const dx = robot.targetX - robot.x;
    const dy = robot.targetY - robot.y;

    if (Math.abs(dx) < 0.01 && Math.abs(dy) < 0.01) {
        pathIndex = (pathIndex + 1) % path.length;
        const nextTarget = path[pathIndex];
        robot.targetX = nextTarget.x;
        robot.targetY = nextTarget.y;

        const angleDx = robot.targetX - robot.x;
        const angleDy = robot.targetY - robot.y;
        robot.angle = Math.atan2(angleDy, angleDx) + Math.PI / 2;
    } else {
        robot.x += dx * animationSpeed;
        robot.y += dy * animationSpeed;
    }
}

function gameLoop() {
    updateRobot();

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    drawGrid();
    drawRobot();

    requestAnimationFrame(gameLoop);
}

// --- Start ---
gameLoop();
