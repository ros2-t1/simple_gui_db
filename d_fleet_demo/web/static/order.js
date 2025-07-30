document.addEventListener('DOMContentLoaded', () => {
    const itemsContainer = document.getElementById('items-container');
    const orderButton = document.getElementById('order-button');
    const orderBtnText = document.getElementById('order-btn-text');
    const robotStatusDiv = document.getElementById('robot-status');
    const statusTextSpan = document.getElementById('status-text');
    const modalConfirmButton = document.getElementById('modal-confirm-button');
    const modalLaterButton = document.getElementById('modal-later-button');
    const userNameSpan = document.getElementById('user-name');
    const logoutBtn = document.getElementById('logout-btn');
    
    let itemsData = [];
    let statusPollingInterval = null;
    let quantities = {};

    // 성공 알림 표시 (먼저 정의)
    function showSuccessNotification(message) {
        const notification = document.createElement('div');
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            background: linear-gradient(135deg, #4CAF50, #45a049);
            color: white;
            padding: 15px 25px;
            border-radius: 10px;
            box-shadow: 0 4px 15px rgba(76, 175, 80, 0.3);
            z-index: 9999;
            font-weight: 600;
            animation: slideDown 0.3s ease-out;
        `;
        notification.textContent = message;
        
        // 애니메이션 CSS 추가
        const style = document.createElement('style');
        style.textContent = `
            @keyframes slideDown {
                from { transform: translateX(-50%) translateY(-100%); opacity: 0; }
                to { transform: translateX(-50%) translateY(0); opacity: 1; }
            }
        `;
        document.head.appendChild(style);
        
        document.body.appendChild(notification);
        
        // 3초 후 제거
        setTimeout(() => {
            notification.remove();
            style.remove();
        }, 3000);
    }

    // 사용자 정보 로드 및 인증 확인
    function loadUserInfo() {
        const residentId = localStorage.getItem('resident_id');
        const userName = localStorage.getItem('user_name');
        
        if (!residentId || !userName) {
            // 로그인 정보가 없으면 로그인 페이지로 리다이렉트
            showSuccessNotification('로그인이 필요합니다. 로그인 페이지로 이동합니다.');
            setTimeout(() => {
                window.location.href = '/login';
            }, 1500);
            return false;
        }
        
        // 사용자 이름 표시
        userNameSpan.textContent = userName;
        return true;
    }

    // 로그아웃 함수
    function logout() {
        const userName = localStorage.getItem('user_name');
        
        // 로컬 스토리지 정리
        localStorage.removeItem('resident_id');
        localStorage.removeItem('user_name');
        
        // 상태 폴링 중지
        if (statusPollingInterval) {
            clearInterval(statusPollingInterval);
        }
        
        // 성공 메시지와 함께 로그인 페이지로 이동
        showSuccessNotification(`${userName}님, 안전하게 로그아웃되었습니다. 👋`);
        
        setTimeout(() => {
            window.location.href = '/login';
        }, 1500);
    }

    // 로그아웃 버튼 이벤트
    logoutBtn.addEventListener('click', () => {
        const userName = localStorage.getItem('user_name');
        
        // 확인 다이얼로그
        if (confirm(`${userName}님, 정말 로그아웃하시겠습니까?`)) {
            logout();
        }
    });

    // 페이지 로드 시 사용자 정보 확인
    if (!loadUserInfo()) {
        return; // 로그인 정보가 없으면 나머지 코드 실행 중지
    }

    // 상품 목록 가져오기
    fetch('/items')
        .then(response => response.json())
        .then(items => {
            itemsData = items;
            items.forEach(item => {
                quantities[item.id] = 0; // 초기 수량 0
                
                const itemDiv = document.createElement('div');
                itemDiv.classList.add('item-card');
                itemDiv.innerHTML = `
                    <div class="item-info">
                        <span class="item-name">${item.type}</span>
                        <span class="item-stock">재고 ${item.quantity}개</span>
                    </div>
                    <div class="quantity-control">
                        <button class="quantity-btn" onclick="updateQuantity(${item.id}, -1)" data-item="${item.id}" data-action="decrease">
                            <i class="bi bi-dash"></i>
                        </button>
                        <div class="quantity-display" id="quantity-${item.id}">0</div>
                        <button class="quantity-btn" onclick="updateQuantity(${item.id}, 1)" data-item="${item.id}" data-action="increase">
                            <i class="bi bi-plus"></i>
                        </button>
                    </div>
                `;
                itemsContainer.appendChild(itemDiv);
            });
            updateOrderButton();
        })
        .catch(error => {
            console.error('상품 목록 로딩 실패:', error);
            itemsContainer.innerHTML = '<p class="text-center text-muted">상품을 불러올 수 없습니다.</p>';
        });

    // 수량 업데이트 함수
    window.updateQuantity = function(itemId, change) {
        const item = itemsData.find(i => i.id === itemId);
        if (!item) return;

        const newQuantity = quantities[itemId] + change;
        
        // 0 미만이거나 재고보다 많으면 return
        if (newQuantity < 0 || newQuantity > item.quantity) return;
        
        quantities[itemId] = newQuantity;
        
        // UI 업데이트
        const quantityDisplay = document.getElementById(`quantity-${itemId}`);
        quantityDisplay.textContent = newQuantity;
        
        // 버튼 상태 업데이트
        const decreaseBtn = document.querySelector(`[data-item="${itemId}"][data-action="decrease"]`);
        const increaseBtn = document.querySelector(`[data-item="${itemId}"][data-action="increase"]`);
        
        decreaseBtn.disabled = newQuantity <= 0;
        increaseBtn.disabled = newQuantity >= item.quantity;
        
        updateOrderButton();
    };

    // 주문 버튼 상태 업데이트
    function updateOrderButton() {
        const totalItems = Object.values(quantities).reduce((sum, qty) => sum + qty, 0);
        
        if (totalItems > 0) {
            orderButton.disabled = false;
            orderBtnText.textContent = `${totalItems}개 주문하기`;
        } else {
            orderButton.disabled = true;
            orderBtnText.textContent = '상품을 선택해주세요';
        }
    }

    // 로봇 상태 폴링 함수
    function pollRobotStatus() {
        const residentId = localStorage.getItem('resident_id');
        const url = residentId ? `/robot_status/robot_1?resident_id=${residentId}` : '/robot_status/robot_1';
        
        fetch(url)
            .then(response => response.json())
            .then(data => {
                console.log('Robot status response:', data);
                console.log('Detailed status info:', {
                    status: data.status,
                    is_waiting_confirm: data.is_waiting_confirm,
                    is_my_order: data.is_my_order,
                    current_task_id: data.current_task_id
                });
                if (data.success) {
                    updateRobotStatus(data.status, data.is_waiting_confirm);
                } else {
                    console.error('Robot status API failed:', data);
                }
            })
            .catch(error => {
                console.error('로봇 상태 확인 실패:', error);
            });
    }

    // 전역 변수로 모달 상태 추적
    let currentModal = null;
    let isModalShown = false;
    let lastWaitingConfirmStatus = false; // 이전 waiting_confirm 상태 추적

    // 로봇 상태 업데이트 함수
    function updateRobotStatus(status, isWaitingConfirm) {
        console.log('Updating robot status:', status, 'isWaitingConfirm:', isWaitingConfirm, 'lastWaitingConfirmStatus:', lastWaitingConfirmStatus, 'isModalShown:', isModalShown);
        
        const statusMap = {
            'idle': '🤖 대기 중',
            'moving_to_arm': '🚀 로봇암으로 이동 중',
            'picking': '🦾 상품 픽업 중',
            'moving_to_user': '🚚 배달 중',
            'waiting_confirm': '📍 도착 - 수령 대기',
            'returning_to_dock': '🔄 복귀 중'
        };

        const displayStatus = statusMap[status] || status;
        statusTextSpan.innerHTML = displayStatus;

        // 로봇이 작업 중일 때만 상태 표시
        if (status !== 'idle') {
            robotStatusDiv.style.display = 'block';
        } else {
            robotStatusDiv.style.display = 'none';
        }

        // 로봇이 도착하면 팝업 표시 (상태 변화 감지 + 중복 방지)
        console.log('Modal logic check:', {
            isWaitingConfirm,
            lastWaitingConfirmStatus,
            isModalShown,
            shouldShow: isWaitingConfirm && !lastWaitingConfirmStatus && !isModalShown
        });
        
        if (isWaitingConfirm && !lastWaitingConfirmStatus && !isModalShown) {
            console.log('Robot just arrived (waiting_confirm changed from false to true), showing modal!');
            showDeliveryModal();
        } else if (!isWaitingConfirm && isModalShown) {
            // waiting_confirm이 아닌 상태가 되면 모달 숨김 (로봇이 복귀 중일 때)
            console.log('Robot is no longer waiting, hiding modal if shown');
            hideDeliveryModal();
        }
        
        // 디버깅용: isWaitingConfirm이 true면 강제로 모달 표시 (임시)
        if (isWaitingConfirm && !isModalShown) {
            console.log('DEBUG: Force showing modal because isWaitingConfirm is true');
            showDeliveryModal();
        }
        
        // 이전 상태 업데이트
        lastWaitingConfirmStatus = isWaitingConfirm;
    }

    // 배달 도착 모달 표시
    function showDeliveryModal() {
        if (isModalShown) {
            console.log('Modal already shown, skipping...');
            return;
        }

        const modalElement = document.getElementById('deliveryModal');
        
        // 기존 모달 인스턴스 제거
        if (currentModal) {
            currentModal.dispose();
            currentModal = null;
        }
        
        currentModal = new bootstrap.Modal(modalElement, {
            backdrop: 'static',
            keyboard: false
        });
        
        // 모달 표시
        currentModal.show();
        isModalShown = true;
        
        // 모달이 숨겨질 때 상태 초기화
        modalElement.addEventListener('hidden.bs.modal', () => {
            isModalShown = false;
            console.log('Modal hidden event fired, status reset');
            
            // 모달 인스턴스 정리
            if (currentModal) {
                currentModal.dispose();
                currentModal = null;
            }
        }, { once: true });
        
        console.log('Modal shown successfully');
    }

    // 배달 도착 모달 숨기기
    function hideDeliveryModal() {
        if (!isModalShown && !currentModal) {
            console.log('Modal not shown or no current modal instance');
            return;
        }
        
        console.log('Hiding delivery modal');
        
        if (currentModal) {
            try {
                currentModal.hide();
            } catch (error) {
                console.error('Error hiding modal:', error);
            }
        }
        
        // 강제로 상태 초기화
        isModalShown = false;
        // lastWaitingConfirmStatus는 건드리지 않음 - 실제 로봇 상태와 동기화 유지
        
        // DOM에서 직접 모달 숨기기 (백업)
        const modalElement = document.getElementById('deliveryModal');
        if (modalElement && modalElement.classList.contains('show')) {
            modalElement.classList.remove('show');
            modalElement.style.display = 'none';
            
            // 백드롭 제거
            const backdrop = document.querySelector('.modal-backdrop');
            if (backdrop) {
                backdrop.remove();
            }
            
            // body 클래스 정리
            document.body.classList.remove('modal-open');
            document.body.style.overflow = '';
            document.body.style.paddingRight = '';
        }
    }

    // 모달의 수령완료 버튼 이벤트
    modalConfirmButton.addEventListener('click', () => {
        // 버튼을 로딩 상태로 변경
        const originalHTML = modalConfirmButton.innerHTML;
        modalConfirmButton.innerHTML = '<div class="loading"></div> 처리 중...';
        modalConfirmButton.disabled = true;

        fetch('/confirm', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({}),
        })
        .then(response => response.json())
        .then(data => {
            // 먼저 버튼 상태 복구
            modalConfirmButton.innerHTML = originalHTML;
            modalConfirmButton.disabled = false;
            
            if (data.error) {
                alert(`수령 실패: ${data.error}`);
            } else {
                // 성공 시 모달 닫기
                console.log('Confirm success, closing modal');
                hideDeliveryModal();
                
                // 성공 알림
                showSuccessNotification('수령 완료 처리되었습니다! ✅');
            }
        })
        .catch(error => {
            // 오류 시에도 버튼 상태 복구
            modalConfirmButton.innerHTML = originalHTML;
            modalConfirmButton.disabled = false;
            
            alert('수령 처리 중 오류가 발생했습니다.');
            console.error('수령 완료 오류:', error);
        });
    });

    // "잠시 후에" 버튼 이벤트
    modalLaterButton.addEventListener('click', () => {
        console.log('User clicked "later" button');
        hideDeliveryModal();
        showSuccessNotification('로봇이 대기 중입니다. 준비되시면 다시 수령해주세요! ⏰');
    });

    // 주문하기 버튼 클릭 이벤트
    orderButton.addEventListener('click', () => {
        const residentId = localStorage.getItem('resident_id');
        // 페이지 로드 시 이미 확인했지만 혹시 모르니 재확인
        if (!residentId) {
            logout();
            return;
        }

        // 선택된 상품들만 필터링
        const orderItems = itemsData.map(item => ({
            id: item.id,
            item_id: item.id,
            quantity: quantities[item.id]
        })).filter(item => item.quantity > 0);

        if (orderItems.length === 0) {
            alert('주문할 상품을 선택해주세요.');
            return;
        }

        // 주문 버튼 로딩 상태
        const originalHTML = orderButton.innerHTML;
        orderButton.innerHTML = '<div class="loading"></div> 주문 중...';
        orderButton.disabled = true;

        fetch('/order', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                resident_id: residentId,
                items: orderItems
            }),
        })
        .then(response => response.json())
        .then(data => {
            if (data.error) {
                alert(`주문 실패: ${data.error}`);
                // 버튼 원래 상태로 복구
                orderButton.innerHTML = originalHTML;
                orderButton.disabled = false;
            } else {
                // 주문 성공 처리
                showSuccessNotification('주문이 접수되었습니다! 🎉');
                
                // 수량 초기화
                Object.keys(quantities).forEach(key => {
                    quantities[key] = 0;
                    const quantityDisplay = document.getElementById(`quantity-${key}`);
                    if (quantityDisplay) quantityDisplay.textContent = '0';
                    
                    // 수량 버튼들도 초기화
                    const decreaseBtn = document.querySelector(`[data-item="${key}"][data-action="decrease"]`);
                    const increaseBtn = document.querySelector(`[data-item="${key}"][data-action="increase"]`);
                    if (decreaseBtn) decreaseBtn.disabled = true;
                    if (increaseBtn) increaseBtn.disabled = false;
                });
                
                // 주문 버튼 원래 상태로 복구
                orderButton.innerHTML = originalHTML;
                orderButton.disabled = true; // 선택된 상품이 없으므로 비활성화
                updateOrderButton(); // 버튼 텍스트 업데이트
                
                // 주문 후 상태 폴링 시작
                if (statusPollingInterval) {
                    clearInterval(statusPollingInterval);
                }
                statusPollingInterval = setInterval(pollRobotStatus, 2000);
                
                // 즉시 한 번 확인
                pollRobotStatus();
            }
        })
        .catch(error => {
            alert('주문 처리 중 오류가 발생했습니다.');
            console.error('주문 오류:', error);
            // 버튼 원래 상태로 복구
            orderButton.innerHTML = originalHTML;
            orderButton.disabled = false;
        });
    });

    // 페이지 로드 시 로봇 상태 한 번 확인
    setTimeout(pollRobotStatus, 1000);
});