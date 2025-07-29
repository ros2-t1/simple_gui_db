document.addEventListener('DOMContentLoaded', () => {
    const form = document.getElementById('login-form');
    const msg = document.getElementById('msg');

    form.addEventListener('submit', async (e) => {
        e.preventDefault();
        const loginId = document.getElementById('login_id').value;
        const password = document.getElementById('password').value;

        try {
            const response = await fetch('/login', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ login_id: loginId, password: password }),
            });

            const data = await response.json();

            if (response.ok) {
                localStorage.setItem('resident_id', data.resident_id);
                localStorage.setItem('user_name', data.name);
                alert(`'${data.name}'님 환영합니다.`);
                window.location.href = '/order';
            } else {
                msg.textContent = data.error || '로그인에 실패했습니다.';
            }
        } catch (error) {
            console.error('Error:', error);
            msg.textContent = '오류가 발생했습니다.';
        }
    });
});