"""
Lightweight launcher that shows a splash screen with a small indeterminate progress bar,
then imports the heavy Qt/PyVista app. Use as the PyInstaller entrypoint for faster
perceived startup without showing a terminal.
"""
import os
import sys
from PyQt5 import QtCore, QtGui, QtWidgets

import config


class LoadingSplash(QtWidgets.QWidget):
    """Floating splash with soft shadow and indeterminate bar; no window frame/box."""

    def __init__(self):
        super().__init__(flags=QtCore.Qt.FramelessWindowHint | QtCore.Qt.SplashScreen | QtCore.Qt.WindowStaysOnTopHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self.setFixedSize(360, 240)

        # Outer layout keeps the translucent margin
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(16, 16, 16, 16)

        # Card container with shadow
        card = QtWidgets.QFrame()
        card.setObjectName("Card")
        card.setStyleSheet("""
            QFrame#Card {
                background: #181818;
                border-radius: 12px;
            }
            QLabel#Title { font-size: 18px; font-weight: 700; color: #f08c28; }
            QLabel#Subtitle { font-size: 11px; color: #b7bcc3; }
            QProgressBar {
                border: 1px solid #2f2f2f;
                border-radius: 6px;
                background: #0f0f0f;
                height: 12px;
            }
            QProgressBar::chunk {
                background: #f08c28;
                border-radius: 6px;
            }
        """)
        shadow = QtWidgets.QGraphicsDropShadowEffect(card)
        shadow.setBlurRadius(28)
        shadow.setOffset(0, 12)
        shadow.setColor(QtGui.QColor(0, 0, 0, 160))
        card.setGraphicsEffect(shadow)

        inner = QtWidgets.QVBoxLayout(card)
        inner.setContentsMargins(24, 24, 24, 24)
        inner.setSpacing(12)

        pixmap = QtGui.QPixmap(config.ICON_PATH)
        if pixmap.isNull():
            pixmap = QtGui.QPixmap(96, 96)
            pixmap.fill(QtGui.QColor("#242424"))
        else:
            pixmap = pixmap.scaled(96, 96, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)

        icon_label = QtWidgets.QLabel()
        icon_label.setAlignment(QtCore.Qt.AlignCenter)
        icon_label.setPixmap(pixmap)
        inner.addWidget(icon_label, alignment=QtCore.Qt.AlignCenter)

        title = QtWidgets.QLabel(config.APP_NAME)
        title.setObjectName("Title")
        title.setAlignment(QtCore.Qt.AlignCenter)
        inner.addWidget(title)

        subtitle = QtWidgets.QLabel("Loading, please wait...")
        subtitle.setObjectName("Subtitle")
        subtitle.setAlignment(QtCore.Qt.AlignCenter)
        inner.addWidget(subtitle)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 0)  # indeterminate
        inner.addWidget(self.progress)

        outer.addWidget(card)

        self._anim_timer = QtCore.QTimer(self)
        self._anim_timer.timeout.connect(lambda: None)  # keep UI pumping
        self._anim_timer.start(100)

        # Center on screen
        screen = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.move(
            screen.center().x() - self.width() // 2,
            screen.center().y() - self.height() // 2,
        )


def main():
    app = QtWidgets.QApplication(sys.argv)
    splash = LoadingSplash()
    splash.show()
    app.processEvents()

    # Heavy imports happen after splash is visible
    from gui_qt import QtControlPanel, apply_dark_palette

    apply_dark_palette(app)
    win = QtControlPanel()
    win.show()
    splash.close()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
