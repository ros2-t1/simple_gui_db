document.addEventListener('DOMContentLoaded', () => {
    const itemsContainer = document.getElementById('items-container');
    const orderButton = document.getElementById('order-button');
    const confirmButton = document.getElementById('confirm-button');
    let itemsData = [];

    // 상품 목록 가져오기
    fetch('/items')
        .then(response => response.json())
        .then(items => {
            itemsData = items;
            items.forEach(item => {
                const itemDiv = document.createElement('div');
                itemDiv.classList.add('item-card'); // Add Bootstrap class
                itemDiv.innerHTML = `
                    <div>
                        <span>${item.type}</span>
                        <span class="text-muted">(재고: ${item.quantity})</span>
                    </div>
                    <div class="input-group input-group-sm w-auto">
                        <span class="input-group-text">수량</span>
                        <input type="number" class="form-control" id="item-${item.id}" min="0" max="${item.quantity}" value="0">
                    </div>
                `;
                itemsContainer.appendChild(itemDiv);
            });
        });

    confirmButton.addEventListener('click', () => {
        fetch('/confirm', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({}),
        })
        .then(response => response.json())
        .then(data => {
            if (data.error) {
                alert(`수령 실패: ${data.error}`);
            } else {
                alert('수령 완료 처리되었습니다!');
            }
        });
    });

    // 주문하기 버튼 클릭 이벤트
    orderButton.addEventListener('click', () => {
        const residentId = localStorage.getItem('resident_id');
        if (!residentId) {
            alert('로그인이 필요합니다.');
            window.location.href = '/login';
            return;
        }

        const orderItems = itemsData.map(item => {
            const quantityInput = document.getElementById(`item-${item.id}`);
            return {
                id: item.id,
                quantity: parseInt(quantityInput.value)
            };
        }).filter(item => item.quantity > 0);

        if (orderItems.length === 0) {
            alert('주문할 상품을 선택해주세요.');
            return;
        }

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
            } else {
                alert(`주문 성공! 응답 데이터: ${JSON.stringify(data)}`);
                // console.log('주문 성공 응답:', data);

                window.location.reload();
            }
        });
    });
});