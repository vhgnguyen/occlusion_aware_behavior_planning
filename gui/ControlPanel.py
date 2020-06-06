from PyQt5 import QtCore
from PyQt5.QtWidgets import (QGroupBox, QHBoxLayout, QPushButton, QGridLayout)

from BirdEyeView import LegendWidget


class ControlPanel(QGroupBox):

    def __init__(self, core, birdEyeView, infoPanel, parent=None):
        super(ControlPanel, self).__init__(parent)

        self.core = core
        self.birdEyeView = birdEyeView
        self.plots_refresh_timer = None

        self.infoPanel = infoPanel
        self.setMaximumSize(700, 200)
        self.setMinimumSize(700, 100)
        
        self.mainLayout = QHBoxLayout()
        self.addLegendWidget()
        self.addControlBox()
        self.mainLayout.addStretch()
        self.setLayout(self.mainLayout)

    def addLegendWidget(self):
        self.legendWidget = LegendWidget()
        self.mainLayout.addWidget(self.legendWidget)

    def addControlBox(self):
        # Button box
        self.buttonBox = QGroupBox()
        self.buttonBox.setTitle("Control buttons")
        self.buttonGrid = QGridLayout()

        # refresh view button
        self.refreshButton = QPushButton("Refresh")
        self.refreshButton.setMaximumWidth(200)
        self.refreshButton.clicked.connect(self.on_refreshButton_clicked)
        self.buttonGrid.addWidget(self.refreshButton, 0, 0)

        # restart button
        self.restartButton = QPushButton("Restart")
        self.restartButton.setStyleSheet('QPushButton {color: blue;}')
        self.restartButton.setMaximumWidth(200)
        self.restartButton.clicked.connect(self.on_restartButton_clicked)
        self.buttonGrid.addWidget(self.restartButton, 0, 1)

        # replay button
        self.replayButton = QPushButton("Replay")
        self.replayButton.setMaximumWidth(200)
        self.replayButton.clicked.connect(self.on_replayButton_clicked)
        self.buttonGrid.addWidget(self.replayButton, 0, 2)

        # simulation button
        self.simulationButton = QPushButton("Start")
        self.simulationButton.setStyleSheet('QPushButton {color: red;}')
        self.simulationButton.setMaximumWidth(200)
        self.simulationButton.clicked.connect(self.on_simulationButton_clicked)
        self.buttonGrid.addWidget(self.simulationButton, 1, 0)

        # move button
        self.moveButton = QPushButton("Next state")
        self.moveButton.setMaximumWidth(200)
        self.moveButton.clicked.connect(self.on_moveButton_clicked)
        self.buttonGrid.addWidget(self.moveButton, 1, 2)

        # stop button
        self.stopSimulationButton = QPushButton("Pause")
        self.stopSimulationButton.setMaximumWidth(200)
        self.stopSimulationButton.clicked.connect(self.on_stopSimulation_clicked)
        self.buttonGrid.addWidget(self.stopSimulationButton, 1, 1)

        self.buttonBox.setLayout(self.buttonGrid)
        self.mainLayout.addWidget(self.buttonBox)

    def on_refreshButton_clicked(self):
        self.birdEyeView.update()
        self.update()

    def on_restartButton_clicked(self):
        self.core.restart()
        self.plots_refresh_timer = QtCore.QTimer()
        self.plots_refresh_timer.setSingleShot(False)
        self.plots_refresh_timer.timeout.connect(self.on_simulation)
        self.plots_refresh_timer.setInterval(99)
        self.plots_refresh_timer.start()

    def on_simulationButton_clicked(self):
        self.plots_refresh_timer = QtCore.QTimer()
        self.plots_refresh_timer.setSingleShot(False)
        self.plots_refresh_timer.timeout.connect(self.on_simulation)
        self.plots_refresh_timer.setInterval(99)
        self.plots_refresh_timer.start()

    def on_replayButton_clicked(self):
        self.replay_refresh_timer = QtCore.QTimer()
        self.replay_refresh_timer.setSingleShot(False)
        self.replay_refresh_timer.timeout.connect(self.on_replay)
        self.replay_refresh_timer.setInterval(99)
        self.replay_refresh_timer.start()

    def on_stopSimulation_clicked(self):
        if self.plots_refresh_timer is not None:
            self.plots_refresh_timer.stop()

    def on_moveButton_clicked(self):
        self.core.move()
        self.birdEyeView.update()
        self.infoPanel.update()

    def on_simulation(self):
        isMove = self.core.move()
        if isMove:
            self.birdEyeView.update()
            self.infoPanel.update()
        else:
            self.plots_refresh_timer.stop()

    def on_replay(self):
        isMove = self.core.replay()
        if isMove:
            self.birdEyeView.update()
            self.infoPanel.update()
        else:
            self.replay_refresh_timer.stop()
    