import React, { useRef, useEffect } from 'react';
import { Paper } from '@mui/material';
import * as PIXI from 'pixi.js';
import { DropShadowFilter } from '@pixi/filter-drop-shadow';

const mapData = {
    grid: [
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [1, 1, 0, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 1, 1, 0],
        [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 1, 1, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    ],
    path: [
        { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 }, { x: 3, y: 1 }, { x: 3, y: 2 },
        { x: 4, y: 2 }, { x: 5, y: 2 }, { x: 6, y: 3 }, { x: 7, y: 3 }, { x: 8, y: 4 },
        { x: 9, y: 4 }, { x: 9, y: 5 }, { x: 9, y: 6 }, { x: 8, y: 7 }, { x: 7, y: 7 },
    ]
};

const MapDisplay = () => {
    const pixiContainer = useRef(null);
    const app = useRef(null);

    useEffect(() => {
        const initializePixi = async () => {
            if (pixiContainer.current) {
                const width = pixiContainer.current.clientWidth;
                const height = pixiContainer.current.clientHeight;

                app.current = new PIXI.Application();
                await app.current.init({
                    width: width,
                    height: height,
                    backgroundColor: 0x2c3e50, // Dark background
                    antialias: true, // Smooth edges
                });

                pixiContainer.current.appendChild(app.current.canvas);

                const cellSize = 40; // Still use for internal calculations, but not for visual grid
                const mapContainer = new PIXI.Container();
                app.current.stage.addChild(mapContainer);

                // Draw obstacles
                for (let y = 0; y < mapData.grid.length; y++) {
                    for (let x = 0; x < mapData.grid[0].length; x++) {
                        if (mapData.grid[y][x] === 1) {
                            const obstacle = new PIXI.Graphics();
                            obstacle.beginFill(0x4a627a); // Darker blue-grey for obstacles
                            obstacle.drawRect(x * cellSize, y * cellSize, cellSize, cellSize);
                            obstacle.endFill();
                            // Add subtle shadow for depth
                            obstacle.filters = [new DropShadowFilter({
                                color: 0x000000,
                                alpha: 0.5,
                                distance: 2,
                                rotation: 45,
                                blur: 2,
                            })];
                            mapContainer.addChild(obstacle);
                        }
                    }
                }

                // Robot path trail
                const pathTrail = new PIXI.Graphics();
                mapContainer.addChild(pathTrail);
                pathTrail.lineStyle(2, 0x1abc9c, 0.7); // Teal, slightly transparent

                // Robot
                const robot = new PIXI.Graphics();
                robot.beginFill(0x1abc9c); // Teal for robot
                robot.drawCircle(0, 0, cellSize * 0.35);
                robot.endFill();
                // Direction pointer
                robot.beginFill(0xffffff); // White pointer
                robot.drawPolygon([
                    0, -cellSize * 0.2,
                    -cellSize * 0.1,
                    cellSize * 0.1, 0,
                ]);
                robot.endFill();
                robot.pivot.set(0, 0); // Center pivot for rotation
                mapContainer.addChild(robot);

                let pathIndex = 0;
                let currentRobotX = mapData.path[0].x * cellSize + cellSize / 2;
                let currentRobotY = mapData.path[0].y * cellSize + cellSize / 2;

                robot.x = currentRobotX;
                robot.y = currentRobotY;

                // Draw initial path point
                pathTrail.moveTo(currentRobotX, currentRobotY);

                app.current.ticker.add((delta) => {
                    const target = mapData.path[pathIndex];
                    const targetX = target.x * cellSize + cellSize / 2;
                    const targetY = target.y * cellSize + cellSize / 2;

                    const dx = targetX - currentRobotX;
                    const dy = targetY - currentRobotY;

                    const distance = Math.sqrt(dx * dx + dy * dy);
                    const speed = 3; // Pixels per frame

                    if (distance < speed) {
                        currentRobotX = targetX;
                        currentRobotY = targetY;
                        pathIndex = (pathIndex + 1) % mapData.path.length;
                        // Move to new start point for next segment
                        pathTrail.moveTo(currentRobotX, currentRobotY);
                    } else {
                        const ratio = speed / distance;
                        currentRobotX += dx * ratio;
                        currentRobotY += dy * ratio;
                        pathTrail.lineTo(currentRobotX, currentRobotY);
                    }

                    robot.x = currentRobotX;
                    robot.y = currentRobotY;

                    // Update robot rotation to face direction of movement
                    if (distance > 0) {
                        robot.rotation = Math.atan2(dy, dx) + Math.PI / 2; // Pointing up
                    }
                });

                // Center the map container
                mapContainer.x = (width - mapContainer.width) / 2;
                mapContainer.y = (height - mapContainer.height) / 2;
            }
        };

        initializePixi();

        return () => {
            if (app.current) {
                app.current.destroy(true, true);
                app.current = null;
            }
        };
    }, []);

    return (
        <Paper ref={pixiContainer} sx={{ width: '100%', height: '100%', overflow: 'hidden' }} elevation={3}/>
    );
};

export default MapDisplay;
