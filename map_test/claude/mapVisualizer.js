class MapVisualizer {
    constructor() {
        this.canvas = document.getElementById('mapCanvas');
        this.ctx = this.canvas.getContext('2d');
        this.mapData = null;
        this.robot = {
            x: 0,
            y: 0,
            angle: 0,
            path: [],
            isMoving: false,
            lidarRange: 50,
            sensorData: []
        };
        this.scale = 1;
        this.offsetX = 0;
        this.offsetY = 0;
        
        this.initializeEventListeners();
        this.loadSampleMap();
    }

    initializeEventListeners() {
        document.getElementById('loadMapBtn').addEventListener('click', () => {
            document.getElementById('mapFileInput').click();
        });

        document.getElementById('mapFileInput').addEventListener('change', (e) => {
            this.loadMapFile(e.target.files[0]);
        });

        document.getElementById('startSimBtn').addEventListener('click', () => {
            this.startRobotSimulation();
        });

        document.getElementById('stopSimBtn').addEventListener('click', () => {
            this.stopRobotSimulation();
        });

        this.canvas.addEventListener('click', (e) => {
            this.handleCanvasClick(e);
        });

        this.canvas.addEventListener('wheel', (e) => {
            this.handleZoom(e);
        });
    }

    loadSampleMap() {
        const sampleMap = {
            width: 200,
            height: 150,
            resolution: 0.025,
            origin: [-2.5, -1.875, 0],
            data: this.generateSampleMapData(200, 150)
        };
        
        this.setMapData(sampleMap);
    }

    generateSampleMapData(width, height) {
        const data = new Array(width * height).fill(0);
        
        // 외벽 생성
        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const index = y * width + x;
                if (x === 0 || x === width-1 || y === 0 || y === height-1) {
                    data[index] = 100;
                }
            }
        }
        
        // 창고/공장 레이아웃 생성
        this.createWarehouseLayout(data, width, height);
        
        // 기계/장비 배치
        this.placeMachinery(data, width, height);
        
        // 통로 및 작업 공간 확보
        this.createCorridors(data, width, height);
        
        // 노이즈 추가 (실제 LiDAR 스캔의 불완전성 시뮬레이션)
        this.addScanNoise(data, width, height);
        
        return data;
    }
    
    createWarehouseLayout(data, width, height) {
        // 메인 통로 (세로)
        const mainCorridorX = Math.floor(width / 2);
        for (let y = 10; y < height - 10; y++) {
            for (let x = mainCorridorX - 3; x <= mainCorridorX + 3; x++) {
                if (x >= 0 && x < width) {
                    data[y * width + x] = 0;
                }
            }
        }
        
        // 메인 통로 (가로)
        const mainCorridorY = Math.floor(height / 2);
        for (let x = 10; x < width - 10; x++) {
            for (let y = mainCorridorY - 3; y <= mainCorridorY + 3; y++) {
                if (y >= 0 && y < height) {
                    data[y * width + x] = 0;
                }
            }
        }
        
        // 저장 구역 (선반들)
        this.createStorageAreas(data, width, height);
        
        // 로딩 도크
        this.createLoadingDocks(data, width, height);
    }
    
    createStorageAreas(data, width, height) {
        const shelfWidth = 2;
        const shelfLength = 15;
        const aisleWidth = 4;
        
        // 왼쪽 저장 구역
        for (let row = 0; row < 3; row++) {
            for (let col = 0; col < 2; col++) {
                const startX = 15 + col * (shelfLength + aisleWidth);
                const startY = 15 + row * (shelfWidth + aisleWidth);
                
                for (let y = startY; y < startY + shelfWidth && y < height; y++) {
                    for (let x = startX; x < startX + shelfLength && x < width; x++) {
                        if (x >= 0 && y >= 0) {
                            data[y * width + x] = 100;
                        }
                    }
                }
            }
        }
        
        // 오른쪽 저장 구역
        for (let row = 0; row < 3; row++) {
            for (let col = 0; col < 2; col++) {
                const startX = width - 35 + col * (shelfLength + aisleWidth);
                const startY = 15 + row * (shelfWidth + aisleWidth);
                
                for (let y = startY; y < startY + shelfWidth && y < height; y++) {
                    for (let x = startX; x < startX + shelfLength && x < width; x++) {
                        if (x >= 0 && y >= 0 && x < width && y < height) {
                            data[y * width + x] = 100;
                        }
                    }
                }
            }
        }
    }
    
    createLoadingDocks(data, width, height) {
        // 상단 로딩 도크
        for (let i = 0; i < 3; i++) {
            const dockX = 20 + i * 25;
            for (let y = 1; y < 8; y++) {
                for (let x = dockX; x < dockX + 8; x++) {
                    if (x < width && y < height) {
                        if (y === 1 || y === 7 || x === dockX || x === dockX + 7) {
                            data[y * width + x] = 100;
                        } else {
                            data[y * width + x] = 0;
                        }
                    }
                }
            }
        }
        
        // 하단 로딩 도크
        for (let i = 0; i < 3; i++) {
            const dockX = 20 + i * 25;
            for (let y = height - 8; y < height - 1; y++) {
                for (let x = dockX; x < dockX + 8; x++) {
                    if (x < width && y >= 0) {
                        if (y === height - 8 || y === height - 2 || x === dockX || x === dockX + 7) {
                            data[y * width + x] = 100;
                        } else {
                            data[y * width + x] = 0;
                        }
                    }
                }
            }
        }
    }
    
    placeMachinery(data, width, height) {
        // 포크리프트 충전소
        const chargingStations = [
            {x: 75, y: 15, w: 4, h: 6},
            {x: 75, y: 25, w: 4, h: 6},
            {x: 75, y: 35, w: 4, h: 6}
        ];
        
        chargingStations.forEach(station => {
            for (let y = station.y; y < station.y + station.h; y++) {
                for (let x = station.x; x < station.x + station.w; x++) {
                    if (x < width && y < height) {
                        data[y * width + x] = 100;
                    }
                }
            }
        });
        
        // 컨베이어 벨트 시스템
        const conveyorY = height - 20;
        for (let x = 15; x < width - 15; x++) {
            for (let y = conveyorY; y < conveyorY + 3; y++) {
                if (y < height) {
                    data[y * width + x] = 100;
                }
            }
        }
    }
    
    createCorridors(data, width, height) {
        // 보조 통로들
        const corridors = [
            {x: 10, y: 10, w: width - 20, h: 2},
            {x: 10, y: height - 12, w: width - 20, h: 2}
        ];
        
        corridors.forEach(corridor => {
            for (let y = corridor.y; y < corridor.y + corridor.h; y++) {
                for (let x = corridor.x; x < corridor.x + corridor.w; x++) {
                    if (x < width && y < height) {
                        data[y * width + x] = 0;
                    }
                }
            }
        });
    }
    
    addScanNoise(data, width, height) {
        // LiDAR 스캔의 불완전성 시뮬레이션
        for (let i = 0; i < data.length; i++) {
            if (data[i] === 0 && Math.random() < 0.002) {
                data[i] = -1; // Unknown 영역
            }
            
            // 벽 근처에 약간의 노이즈 추가
            const x = i % width;
            const y = Math.floor(i / width);
            
            if (data[i] === 100) {
                // 주변에 약간의 unknown 영역 추가
                for (let dy = -1; dy <= 1; dy++) {
                    for (let dx = -1; dx <= 1; dx++) {
                        const nx = x + dx;
                        const ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            const ni = ny * width + nx;
                            if (data[ni] === 0 && Math.random() < 0.1) {
                                data[ni] = -1;
                            }
                        }
                    }
                }
            }
        }
    }

    setMapData(mapData) {
        this.mapData = mapData;
        this.robot.x = mapData.width / 2;
        this.robot.y = mapData.height / 2;
        this.robot.path = [];
        
        this.updateMapInfo();
        this.fitMapToCanvas();
        this.render();
    }

    updateMapInfo() {
        if (!this.mapData) return;
        
        document.getElementById('resolution').textContent = this.mapData.resolution.toFixed(3) + ' m/px';
        document.getElementById('mapSize').textContent = `${this.mapData.width} x ${this.mapData.height}`;
        document.getElementById('origin').textContent = 
            `(${this.mapData.origin[0]}, ${this.mapData.origin[1]})`;
    }

    fitMapToCanvas() {
        if (!this.mapData) return;
        
        const padding = 20;
        const scaleX = (this.canvas.width - padding * 2) / this.mapData.width;
        const scaleY = (this.canvas.height - padding * 2) / this.mapData.height;
        
        this.scale = Math.min(scaleX, scaleY);
        this.offsetX = (this.canvas.width - this.mapData.width * this.scale) / 2;
        this.offsetY = (this.canvas.height - this.mapData.height * this.scale) / 2;
    }

    render() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        if (this.mapData) {
            this.renderMap();
            this.renderRobot();
        }
        
        this.renderGrid();
    }

    renderMap() {
        this.ctx.save();
        this.ctx.translate(this.offsetX, this.offsetY);
        this.ctx.scale(this.scale, this.scale);
        
        // 베이스 플로어 렌더링
        this.renderFloor();
        
        // 벽과 장애물 렌더링
        this.renderWallsAndObstacles();
        
        // 산업 시설물 렌더링
        this.renderIndustrialFeatures();
        
        // 바닥 마킹 렌더링
        this.renderFloorMarkings();
        
        this.ctx.restore();
    }
    
    renderFloor() {
        // 콘크리트 바닥 텍스처
        const gradient = this.ctx.createLinearGradient(0, 0, this.mapData.width, this.mapData.height);
        gradient.addColorStop(0, '#f5f5f5');
        gradient.addColorStop(0.3, '#eeeeee');
        gradient.addColorStop(0.7, '#e8e8e8');
        gradient.addColorStop(1, '#e0e0e0');
        
        this.ctx.fillStyle = gradient;
        this.ctx.fillRect(0, 0, this.mapData.width, this.mapData.height);
        
        // 바닥 패턴 (타일 느낌)
        this.ctx.strokeStyle = 'rgba(200, 200, 200, 0.3)';
        this.ctx.lineWidth = 0.5;
        
        const tileSize = 20;
        for (let x = 0; x <= this.mapData.width; x += tileSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.mapData.height);
            this.ctx.stroke();
        }
        for (let y = 0; y <= this.mapData.height; y += tileSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.mapData.width, y);
            this.ctx.stroke();
        }
    }
    
    renderWallsAndObstacles() {
        for (let y = 0; y < this.mapData.height; y++) {
            for (let x = 0; x < this.mapData.width; x++) {
                const index = y * this.mapData.width + x;
                const value = this.mapData.data[index];
                
                if (value === 100) { // 벽/장애물
                    this.renderWallTile(x, y);
                } else if (value === -1) { // Unknown 영역
                    this.renderUnknownTile(x, y);
                }
            }
        }
    }
    
    renderWallTile(x, y) {
        // 벽의 3D 효과를 위한 그라데이션
        const wallGradient = this.ctx.createLinearGradient(x, y, x + 1, y + 1);
        wallGradient.addColorStop(0, '#4a4a4a');
        wallGradient.addColorStop(0.3, '#3a3a3a');
        wallGradient.addColorStop(0.7, '#2a2a2a');
        wallGradient.addColorStop(1, '#1a1a1a');
        
        this.ctx.fillStyle = wallGradient;
        this.ctx.fillRect(x, y, 1, 1);
        
        // 벽의 하이라이트
        this.ctx.fillStyle = 'rgba(100, 100, 100, 0.8)';
        this.ctx.fillRect(x, y, 1, 0.15);
        this.ctx.fillRect(x, y, 0.15, 1);
        
        // 벽의 그림자
        this.ctx.fillStyle = 'rgba(0, 0, 0, 0.4)';
        this.ctx.fillRect(x + 0.85, y + 0.15, 0.15, 0.85);
        this.ctx.fillRect(x + 0.15, y + 0.85, 0.85, 0.15);
    }
    
    renderUnknownTile(x, y) {
        // Unknown 영역을 점선 패턴으로
        const pattern = (x + y) % 4;
        if (pattern < 2) {
            this.ctx.fillStyle = 'rgba(150, 150, 150, 0.6)';
            this.ctx.fillRect(x, y, 1, 1);
        }
    }
    
    renderIndustrialFeatures() {
        // 창고 선반들 (더 현실적으로)
        this.renderWarehouseShelves();
        
        // 기계/장비
        this.renderMachinery();
        
        // 충전 스테이션
        this.renderChargingStations();
        
        // 컨베이어 벨트
        this.renderConveyorBelt();
        
        // 파이프라인
        this.renderPipelines();
    }
    
    renderWarehouseShelves() {
        const shelves = [
            {x: 15, y: 15, w: 15, h: 2, orientation: 'horizontal'},
            {x: 15, y: 19, w: 15, h: 2, orientation: 'horizontal'},
            {x: 15, y: 23, w: 15, h: 2, orientation: 'horizontal'},
            {x: 35, y: 15, w: 15, h: 2, orientation: 'horizontal'},
            {x: 35, y: 19, w: 15, h: 2, orientation: 'horizontal'},
            {x: 35, y: 23, w: 15, h: 2, orientation: 'horizontal'},
            
            {x: this.mapData.width - 50, y: 15, w: 15, h: 2, orientation: 'horizontal'},
            {x: this.mapData.width - 50, y: 19, w: 15, h: 2, orientation: 'horizontal'},
            {x: this.mapData.width - 50, y: 23, w: 15, h: 2, orientation: 'horizontal'},
            {x: this.mapData.width - 30, y: 15, w: 15, h: 2, orientation: 'horizontal'},
            {x: this.mapData.width - 30, y: 19, w: 15, h: 2, orientation: 'horizontal'},
            {x: this.mapData.width - 30, y: 23, w: 15, h: 2, orientation: 'horizontal'},
        ];
        
        shelves.forEach(shelf => {
            // 선반 베이스
            const shelfGradient = this.ctx.createLinearGradient(shelf.x, shelf.y, shelf.x + shelf.w, shelf.y + shelf.h);
            shelfGradient.addColorStop(0, '#8B4513');
            shelfGradient.addColorStop(0.5, '#A0522D');
            shelfGradient.addColorStop(1, '#654321');
            
            this.ctx.fillStyle = shelfGradient;
            this.ctx.fillRect(shelf.x, shelf.y, shelf.w, shelf.h);
            
            // 선반 하이라이트
            this.ctx.fillStyle = 'rgba(160, 82, 45, 0.8)';
            this.ctx.fillRect(shelf.x, shelf.y, shelf.w, 0.3);
            
            // 선반 그림자
            this.ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
            this.ctx.fillRect(shelf.x + 0.2, shelf.y + shelf.h, shelf.w, 0.5);
            
            // 상품 박스들 (랜덤하게)
            for (let i = 0; i < 3; i++) {
                const boxX = shelf.x + 2 + i * 4;
                const boxY = shelf.y - 0.8;
                
                this.ctx.fillStyle = ['#8B4513', '#CD853F', '#DEB887'][Math.floor(Math.random() * 3)];
                this.ctx.fillRect(boxX, boxY, 2, 0.8);
                
                // 박스 하이라이트
                this.ctx.fillStyle = 'rgba(255, 255, 255, 0.3)';
                this.ctx.fillRect(boxX, boxY, 2, 0.2);
            }
        });
    }
    
    renderMachinery() {
        // 대형 기계들
        const machines = [
            {x: 75, y: 45, w: 8, h: 12, type: 'compressor'},
            {x: 85, y: 45, w: 6, h: 10, type: 'control_panel'},
            {x: 75, y: 60, w: 10, h: 8, type: 'generator'}
        ];
        
        machines.forEach(machine => {
            // 기계 베이스
            const machineGradient = this.ctx.createRadialGradient(
                machine.x + machine.w/2, machine.y + machine.h/2, 0,
                machine.x + machine.w/2, machine.y + machine.h/2, Math.max(machine.w, machine.h)/2
            );
            machineGradient.addColorStop(0, '#707070');
            machineGradient.addColorStop(0.7, '#505050');
            machineGradient.addColorStop(1, '#303030');
            
            this.ctx.fillStyle = machineGradient;
            this.ctx.fillRect(machine.x, machine.y, machine.w, machine.h);
            
            // 기계 디테일
            this.ctx.fillStyle = '#FFD700'; // 경고 표시
            this.ctx.fillRect(machine.x + 1, machine.y + 1, 1, 1);
            
            this.ctx.fillStyle = '#FF4500'; // 전원 표시
            this.ctx.beginPath();
            this.ctx.arc(machine.x + machine.w - 2, machine.y + 2, 0.5, 0, 2 * Math.PI);
            this.ctx.fill();
            
            // 그림자
            this.ctx.fillStyle = 'rgba(0, 0, 0, 0.4)';
            this.ctx.fillRect(machine.x + 1, machine.y + machine.h, machine.w, 1);
        });
    }
    
    renderChargingStations() {
        const stations = [
            {x: 160, y: 20, w: 4, h: 6},
            {x: 160, y: 30, w: 4, h: 6},
            {x: 160, y: 40, w: 4, h: 6}
        ];
        
        stations.forEach(station => {
            // 충전소 베이스
            this.ctx.fillStyle = '#32CD32';
            this.ctx.fillRect(station.x, station.y, station.w, station.h);
            
            // 충전 케이블
            this.ctx.strokeStyle = '#000000';
            this.ctx.lineWidth = 0.5;
            this.ctx.beginPath();
            this.ctx.moveTo(station.x + station.w/2, station.y + station.h);
            this.ctx.lineTo(station.x + station.w/2 + 2, station.y + station.h + 2);
            this.ctx.stroke();
            
            // LED 표시
            this.ctx.fillStyle = '#00FF00';
            this.ctx.beginPath();
            this.ctx.arc(station.x + station.w/2, station.y + 1, 0.3, 0, 2 * Math.PI);
            this.ctx.fill();
        });
    }
    
    renderConveyorBelt() {
        const beltY = this.mapData.height - 25;
        const beltWidth = this.mapData.width - 30;
        
        // 컨베이어 베이스
        const beltGradient = this.ctx.createLinearGradient(15, beltY, 15, beltY + 4);
        beltGradient.addColorStop(0, '#2F4F4F');
        beltGradient.addColorStop(0.5, '#1C1C1C');
        beltGradient.addColorStop(1, '#000000');
        
        this.ctx.fillStyle = beltGradient;
        this.ctx.fillRect(15, beltY, beltWidth, 4);
        
        // 컨베이어 롤러들
        for (let x = 20; x < 15 + beltWidth; x += 10) {
            this.ctx.fillStyle = '#696969';
            this.ctx.beginPath();
            this.ctx.arc(x, beltY + 2, 1, 0, 2 * Math.PI);
            this.ctx.fill();
        }
        
        // 이동하는 박스들 (애니메이션 효과)
        const time = Date.now() / 1000;
        for (let i = 0; i < 3; i++) {
            const boxX = (20 + i * 30 + time * 10) % beltWidth + 15;
            this.ctx.fillStyle = '#8B4513';
            this.ctx.fillRect(boxX, beltY - 2, 3, 2);
            
            // 박스 하이라이트
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.3)';
            this.ctx.fillRect(boxX, beltY - 2, 3, 0.5);
        }
    }
    
    renderPipelines() {
        // 천장 파이프라인
        this.ctx.strokeStyle = '#4682B4';
        this.ctx.lineWidth = 2;
        
        // 메인 파이프라인
        this.ctx.beginPath();
        this.ctx.moveTo(10, 60);
        this.ctx.lineTo(this.mapData.width - 10, 60);
        this.ctx.stroke();
        
        // 지관들
        for (let x = 30; x < this.mapData.width - 30; x += 40) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 60);
            this.ctx.lineTo(x, 80);
            this.ctx.stroke();
            
            // 밸브
            this.ctx.fillStyle = '#FF6347';
            this.ctx.beginPath();
            this.ctx.arc(x, 70, 1.5, 0, 2 * Math.PI);
            this.ctx.fill();
        }
    }
    
    renderFloorMarkings() {
        // 안전 라인 (노란색)
        this.ctx.strokeStyle = '#FFD700';
        this.ctx.lineWidth = 1;
        this.ctx.setLineDash([5, 5]);
        
        // 메인 통로 안전선
        const mainCorridorX = Math.floor(this.mapData.width / 2);
        this.ctx.beginPath();
        this.ctx.moveTo(mainCorridorX - 5, 10);
        this.ctx.lineTo(mainCorridorX - 5, this.mapData.height - 10);
        this.ctx.stroke();
        
        this.ctx.beginPath();
        this.ctx.moveTo(mainCorridorX + 5, 10);
        this.ctx.lineTo(mainCorridorX + 5, this.mapData.height - 10);
        this.ctx.stroke();
        
        // 가로 통로 안전선
        const mainCorridorY = Math.floor(this.mapData.height / 2);
        this.ctx.beginPath();
        this.ctx.moveTo(10, mainCorridorY - 5);
        this.ctx.lineTo(this.mapData.width - 10, mainCorridorY - 5);
        this.ctx.stroke();
        
        this.ctx.beginPath();
        this.ctx.moveTo(10, mainCorridorY + 5);
        this.ctx.lineTo(this.mapData.width - 10, mainCorridorY + 5);
        this.ctx.stroke();
        
        this.ctx.setLineDash([]);
        
        // 방향 표시 화살표들
        this.renderDirectionArrows();
        
        // 구역 번호
        this.renderZoneNumbers();
    }
    
    renderDirectionArrows() {
        this.ctx.fillStyle = '#FFD700';
        
        const arrows = [
            {x: 50, y: this.mapData.height/2, dir: 0}, // 오른쪽
            {x: this.mapData.width - 50, y: this.mapData.height/2, dir: Math.PI}, // 왼쪽
            {x: this.mapData.width/2, y: 30, dir: Math.PI/2}, // 아래
            {x: this.mapData.width/2, y: this.mapData.height - 30, dir: -Math.PI/2} // 위
        ];
        
        arrows.forEach(arrow => {
            this.ctx.save();
            this.ctx.translate(arrow.x, arrow.y);
            this.ctx.rotate(arrow.dir);
            
            this.ctx.beginPath();
            this.ctx.moveTo(3, 0);
            this.ctx.lineTo(-2, -2);
            this.ctx.lineTo(-2, 2);
            this.ctx.closePath();
            this.ctx.fill();
            
            this.ctx.restore();
        });
    }
    
    renderZoneNumbers() {
        this.ctx.fillStyle = '#000080';
        this.ctx.font = '6px Arial';
        this.ctx.textAlign = 'center';
        
        const zones = [
            {x: 25, y: 20, text: 'A1'},
            {x: 45, y: 20, text: 'A2'},
            {x: this.mapData.width - 45, y: 20, text: 'B1'},
            {x: this.mapData.width - 25, y: 20, text: 'B2'},
            {x: 80, y: 50, text: 'M1'}, // 기계 구역
            {x: this.mapData.width/2, y: this.mapData.height - 15, text: 'CV1'} // 컨베이어 구역
        ];
        
        zones.forEach(zone => {
            // 배경 원
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
            this.ctx.beginPath();
            this.ctx.arc(zone.x, zone.y, 3, 0, 2 * Math.PI);
            this.ctx.fill();
            
            // 텍스트
            this.ctx.fillStyle = '#000080';
            this.ctx.fillText(zone.text, zone.x, zone.y + 1);
        });
    }

    renderRobotPath() {
        if (this.robot.path.length < 2) return;
        
        this.ctx.save();
        this.ctx.translate(this.offsetX, this.offsetY);
        this.ctx.scale(this.scale, this.scale);
        
        this.ctx.strokeStyle = '#ff6b6b';
        this.ctx.lineWidth = 2 / this.scale;
        this.ctx.setLineDash([5 / this.scale, 5 / this.scale]);
        
        this.ctx.beginPath();
        this.ctx.moveTo(this.robot.path[0].x, this.robot.path[0].y);
        for (let i = 1; i < this.robot.path.length; i++) {
            this.ctx.lineTo(this.robot.path[i].x, this.robot.path[i].y);
        }
        this.ctx.stroke();
        
        this.ctx.restore();
    }

    renderLidarScan() {
        this.updateLidarData();
        
        this.ctx.save();
        this.ctx.translate(this.offsetX, this.offsetY);
        this.ctx.scale(this.scale, this.scale);
        
        // LiDAR 스캔 범위 표시
        this.ctx.strokeStyle = 'rgba(0, 255, 136, 0.1)';
        this.ctx.lineWidth = 1 / this.scale;
        this.ctx.beginPath();
        this.ctx.arc(this.robot.x, this.robot.y, this.robot.lidarRange, 0, 2 * Math.PI);
        this.ctx.stroke();
        
        // LiDAR 포인트들
        this.ctx.fillStyle = 'rgba(0, 255, 136, 0.8)';
        this.robot.sensorData.forEach(point => {
            this.ctx.beginPath();
            this.ctx.arc(point.x, point.y, 0.5 / this.scale, 0, 2 * Math.PI);
            this.ctx.fill();
        });
        
        this.ctx.restore();
    }
    
    updateLidarData() {
        this.robot.sensorData = [];
        const numRays = 360;
        
        for (let i = 0; i < numRays; i += 2) {
            const angle = (i * Math.PI) / 180;
            const rayX = Math.cos(angle);
            const rayY = Math.sin(angle);
            
            for (let dist = 1; dist < this.robot.lidarRange; dist += 0.5) {
                const checkX = Math.floor(this.robot.x + rayX * dist);
                const checkY = Math.floor(this.robot.y + rayY * dist);
                
                if (checkX < 0 || checkX >= this.mapData.width || 
                    checkY < 0 || checkY >= this.mapData.height) break;
                
                const index = checkY * this.mapData.width + checkX;
                if (this.mapData.data[index] === 100) {
                    // 약간의 노이즈 추가
                    const noiseX = (Math.random() - 0.5) * 0.5;
                    const noiseY = (Math.random() - 0.5) * 0.5;
                    
                    this.robot.sensorData.push({
                        x: checkX + noiseX,
                        y: checkY + noiseY
                    });
                    break;
                }
            }
        }
    }

    renderRobot() {
        this.ctx.save();
        this.ctx.translate(this.offsetX, this.offsetY);
        this.ctx.scale(this.scale, this.scale);
        
        this.ctx.translate(this.robot.x, this.robot.y);
        this.ctx.rotate(this.robot.angle);
        
        // 로봇 그림자
        this.ctx.fillStyle = 'rgba(0, 0, 0, 0.2)';
        this.ctx.beginPath();
        this.ctx.arc(1, 1, 4, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // 로봇 메인 바디
        this.ctx.fillStyle = '#00ff88';
        this.ctx.strokeStyle = '#00cc6a';
        this.ctx.lineWidth = 1.5;
        
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 4, 0, 2 * Math.PI);
        this.ctx.fill();
        this.ctx.stroke();
        
        // 방향 표시 화살표
        this.ctx.fillStyle = '#ffffff';
        this.ctx.beginPath();
        this.ctx.moveTo(2.5, 0);
        this.ctx.lineTo(-1.5, -1.5);
        this.ctx.lineTo(-1.5, 1.5);
        this.ctx.closePath();
        this.ctx.fill();
        
        this.ctx.restore();
        
        this.updateRobotInfo();
    }

    renderGrid() {
        if (!this.mapData || this.scale < 2) return;
        
        this.ctx.save();
        this.ctx.strokeStyle = 'rgba(0, 0, 0, 0.1)';
        this.ctx.lineWidth = 0.5;
        
        const startX = this.offsetX;
        const startY = this.offsetY;
        const endX = startX + this.mapData.width * this.scale;
        const endY = startY + this.mapData.height * this.scale;
        
        for (let x = startX; x <= endX; x += this.scale) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, startY);
            this.ctx.lineTo(x, endY);
            this.ctx.stroke();
        }
        
        for (let y = startY; y <= endY; y += this.scale) {
            this.ctx.beginPath();
            this.ctx.moveTo(startX, y);
            this.ctx.lineTo(endX, y);
            this.ctx.stroke();
        }
        
        this.ctx.restore();
    }

    updateRobotInfo() {
        const worldX = (this.robot.x * this.mapData.resolution + this.mapData.origin[0]).toFixed(3);
        const worldY = (this.robot.y * this.mapData.resolution + this.mapData.origin[1]).toFixed(3);
        const angle = (this.robot.angle * 180 / Math.PI).toFixed(1) + '°';
        const lidarPoints = this.robot.sensorData.length;
        
        document.getElementById('robotPos').textContent = `(${worldX}m, ${worldY}m)`;
        document.getElementById('robotAngle').textContent = angle;
        document.getElementById('robotStatus').textContent = this.robot.isMoving ? 'MOVING' : 'IDLE';
        
        // LiDAR 정보 업데이트
        if (document.getElementById('lidarPoints')) {
            document.getElementById('lidarPoints').textContent = lidarPoints;
        }
    }

    handleCanvasClick(e) {
        if (!this.mapData) return;
        
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offsetX) / this.scale;
        const y = (e.clientY - rect.top - this.offsetY) / this.scale;
        
        if (x >= 0 && x < this.mapData.width && y >= 0 && y < this.mapData.height) {
            this.moveRobotTo(x, y);
        }
    }

    handleZoom(e) {
        e.preventDefault();
        
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        
        const zoom = e.deltaY < 0 ? 1.1 : 0.9;
        const newScale = Math.max(0.1, Math.min(10, this.scale * zoom));
        
        this.offsetX = mouseX - (mouseX - this.offsetX) * (newScale / this.scale);
        this.offsetY = mouseY - (mouseY - this.offsetY) * (newScale / this.scale);
        this.scale = newScale;
        
        this.render();
    }

    moveRobotTo(targetX, targetY) {
        if (this.robot.isMoving) return;
        
        this.robot.path.push({x: this.robot.x, y: this.robot.y});
        this.robot.path.push({x: targetX, y: targetY});
        
        const dx = targetX - this.robot.x;
        const dy = targetY - this.robot.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        const targetAngle = Math.atan2(dy, dx);
        
        this.robot.isMoving = true;
        
        const steps = Math.max(30, Math.floor(distance));
        const stepX = dx / steps;
        const stepY = dy / steps;
        const stepAngle = this.getAngleDifference(this.robot.angle, targetAngle) / steps;
        
        let currentStep = 0;
        
        const animate = () => {
            if (currentStep < steps) {
                this.robot.x += stepX;
                this.robot.y += stepY;
                this.robot.angle += stepAngle;
                this.render();
                currentStep++;
                requestAnimationFrame(animate);
            } else {
                this.robot.x = targetX;
                this.robot.y = targetY;
                this.robot.angle = targetAngle;
                this.robot.isMoving = false;
                this.render();
            }
        };
        
        animate();
    }

    getAngleDifference(current, target) {
        let diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    startRobotSimulation() {
        if (this.robot.isMoving || !this.mapData) return;
        
        const simulateMovement = () => {
            if (!this.robot.isMoving) {
                const targetX = Math.random() * this.mapData.width;
                const targetY = Math.random() * this.mapData.height;
                
                const index = Math.floor(targetY) * this.mapData.width + Math.floor(targetX);
                if (this.mapData.data[index] === 0) {
                    this.moveRobotTo(targetX, targetY);
                }
            }
            
            if (this.simulationRunning) {
                setTimeout(simulateMovement, 2000 + Math.random() * 3000);
            }
        };
        
        this.simulationRunning = true;
        simulateMovement();
    }

    stopRobotSimulation() {
        this.simulationRunning = false;
    }

    loadMapFile(file) {
        if (!file) return;
        
        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                if (file.name.endsWith('.json')) {
                    const mapData = JSON.parse(e.target.result);
                    this.setMapData(mapData);
                } else {
                    console.warn('Unsupported file format. Please use JSON format.');
                }
            } catch (error) {
                console.error('Error loading map file:', error);
            }
        };
        reader.readAsText(file);
    }
}

document.addEventListener('DOMContentLoaded', () => {
    new MapVisualizer();
});