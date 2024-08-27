# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'chessGameWindows.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from qfluentwidgets import PushButton
from qfluentwidgets import TransparentPushButton
from qfluentwidgets import ImageLabel
from qfluentwidgets import BodyLabel


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(834, 666)
        self.imageLabel = ImageLabel(Form)
        self.imageLabel.setObjectName(u"imageLabel")
        self.imageLabel.setGeometry(QRect(40, 30, 600, 600))
        self.splitter_2 = QSplitter(Form)
        self.splitter_2.setObjectName(u"splitter_2")
        self.splitter_2.setGeometry(QRect(690, 30, 121, 101))
        self.splitter_2.setOrientation(Qt.Vertical)
        self.startButton = PushButton(self.splitter_2)
        self.startButton.setObjectName(u"startButton")
        self.splitter_2.addWidget(self.startButton)
        self.ruleButton = TransparentPushButton(self.splitter_2)
        self.ruleButton.setObjectName(u"ruleButton")
        self.splitter_2.addWidget(self.ruleButton)
        self.label = BodyLabel(Form)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(700, 220, 71, 41))
        self.playerName = BodyLabel(Form)
        self.playerName.setObjectName(u"playerName")
        self.playerName.setGeometry(QRect(700, 260, 101, 41))

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.imageLabel.setText(QCoreApplication.translate("Form", u"1", None))
        self.startButton.setText(QCoreApplication.translate("Form", u"\u5f00\u59cb\u6e38\u620f", None))
        self.ruleButton.setText(QCoreApplication.translate("Form", u"\u6e38\u620f\u89c4\u5219", None))
        self.label.setText(QCoreApplication.translate("Form", u"<html><head/><body><p><span style=\" font-size:16pt; font-weight:600;\">Player</span></p></body></html>", None))
        self.playerName.setText(QCoreApplication.translate("Form", u"<html><head/><body><p><span style=\" font-size:22pt; font-weight:600; color:#ef2929;\">Human</span></p></body></html>", None))
    # retranslateUi

