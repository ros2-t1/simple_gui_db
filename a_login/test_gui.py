import sys
import requests
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit,
    QPushButton, QVBoxLayout, QMessageBox
)

class LoginWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("입소자 로그인")

        # UI 요소
        self.label = QLabel("로그인 ID:")
        self.login_input = QLineEdit()
        self.pw_label = QLabel("비밀번호:")
        self.pw_input = QLineEdit()
        self.pw_input.setEchoMode(QLineEdit.Password)

        self.login_btn = QPushButton("로그인")
        self.login_btn.clicked.connect(self.try_login)

        # 레이아웃
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.login_input)
        layout.addWidget(self.pw_label)
        layout.addWidget(self.pw_input)
        layout.addWidget(self.login_btn)
        self.setLayout(layout)

    def try_login(self):
        login_id = self.login_input.text()
        password = self.pw_input.text()

        try:
            res = requests.post("http://20.249.209.1:5000/login", json={
                "login_id": login_id,
                "password": password
            })

            if res.status_code == 200:
                data = res.json()
                print(data)  # 추가
                resident_id = data["resident_id"]
                name = data["name"]
                QMessageBox.information(self, "로그인 성공", f"{name}님 환영합니다! \n ")
                # 메인 화면으로 이동하거나 resident_id 저장 가능
            else:
                QMessageBox.warning(self, "로그인 실패", "ID 또는 비밀번호가 틀렸습니다.")
        except Exception as e:
            QMessageBox.critical(self, "서버 오류", str(e))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = LoginWindow()
    win.show()
    sys.exit(app.exec_())