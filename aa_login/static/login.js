// login.js

document.addEventListener("DOMContentLoaded", () => {
  const form = document.getElementById("login-form");
  const msg = document.getElementById("msg");

  form.addEventListener("submit", async (e) => {
    e.preventDefault();
    const login_id = document.getElementById("login_id").value;
    const password = document.getElementById("password").value;

    const res = await fetch("/login", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ login_id, password }),
    });

    if (res.ok) {
      const data = await res.json();
      alert(`${data.name}님 환영합니다!`);
      window.location.href = `/dashboard?resident_id=${data.resident_id}`;
    } else {
      msg.textContent = "ID 또는 비밀번호가 틀렸습니다.";
    }
  });
});
