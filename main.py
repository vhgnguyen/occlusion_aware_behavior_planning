import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'src/types')
sys.path.insert(0, 'src/stuffs')

sys.path.insert(0, 'gui')

from PyQt5.QtWidgets import QApplication
                             
from core import Core
from MainWindow import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    core = Core()
    GUI = MainWindow(core=core)
    sys.exit(app.exec_())
