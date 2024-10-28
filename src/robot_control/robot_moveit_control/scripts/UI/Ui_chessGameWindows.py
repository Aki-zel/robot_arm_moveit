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
from qfluentwidgets import ImageLabel
from qfluentwidgets import BodyLabel
from qfluentwidgets import PrimaryPushButton


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.setWindowModality(Qt.NonModal)
        Form.resize(1280, 720)
        Form.setMinimumSize(QSize(1280, 720))
        Form.setStyleSheet(u"")
        self.verticalLayout_5 = QVBoxLayout(Form)
        self.verticalLayout_5.setSpacing(0)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.mainWidget = QWidget(Form)
        self.mainWidget.setObjectName(u"mainWidget")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.mainWidget.sizePolicy().hasHeightForWidth())
        self.mainWidget.setSizePolicy(sizePolicy)
        self.mainWidget.setStyleSheet(u"QWidget#mainWidget {\n"
"    border-image: url(/home/yds/Pictures/windows/background.png) 0 0 0 0;  /* \u80cc\u666f\u56fe\u7247\u8def\u5f84 */\n"
"}\n"
"QLabel#iconlabel {\n"
"    background-image: url(/home/yds/Pictures/windows/icons/cloudicon.png) 0 0 0 0;  /* \u80cc\u666f\u56fe\u7247\u8def\u5f84 */\n"
"	background-position: center;  /* \u56fe\u7247\u5c45\u4e2d */\n"
"    background-repeat: no-repeat;  /* \u4e0d\u91cd\u590d */\n"
"    background-size: contain;  /* \u786e\u4fdd\u56fe\u7247\u6839\u636e QLabel \u7684\u5927\u5c0f\u7f29\u653e */\n"
"}\n"
"\n"
"QTableWidget::item:selected {\n"
"        background-color: white;  /* \u8bbe\u7f6e\u9009\u4e2d\u9879\u7684\u80cc\u666f\u989c\u8272 */\n"
"}\n"
"\n"
"")
        self.verticalLayout_17 = QVBoxLayout(self.mainWidget)
        self.verticalLayout_17.setSpacing(0)
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.verticalLayout_17.setContentsMargins(0, 0, 0, 0)
        self.topWidget = QWidget(self.mainWidget)
        self.topWidget.setObjectName(u"topWidget")
        self.horizontalLayout_5 = QHBoxLayout(self.topWidget)
        self.horizontalLayout_5.setSpacing(0)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.iconlabel = QLabel(self.topWidget)
        self.iconlabel.setObjectName(u"iconlabel")
        self.iconlabel.setMinimumSize(QSize(200, 80))

        self.horizontalLayout_5.addWidget(self.iconlabel)

        self.horizontalSpacer_4 = QSpacerItem(1077, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer_4)


        self.verticalLayout_17.addWidget(self.topWidget)

        self.baseWidget = QWidget(self.mainWidget)
        self.baseWidget.setObjectName(u"baseWidget")
        sizePolicy.setHeightForWidth(self.baseWidget.sizePolicy().hasHeightForWidth())
        self.baseWidget.setSizePolicy(sizePolicy)
        self.baseWidget.setMaximumSize(QSize(16777215, 16777215))
        self.baseWidget.setStyleSheet(u"")
        self.horizontalLayout = QHBoxLayout(self.baseWidget)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_3)

        self.leftWidget = QWidget(self.baseWidget)
        self.leftWidget.setObjectName(u"leftWidget")
        sizePolicy.setHeightForWidth(self.leftWidget.sizePolicy().hasHeightForWidth())
        self.leftWidget.setSizePolicy(sizePolicy)
        self.leftWidget.setMaximumSize(QSize(16777215, 16777215))
        self.verticalLayout_4 = QVBoxLayout(self.leftWidget)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.game_one_bt = PrimaryPushButton(self.leftWidget)
        self.game_one_bt.setObjectName(u"game_one_bt")
        sizePolicy1 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.game_one_bt.sizePolicy().hasHeightForWidth())
        self.game_one_bt.setSizePolicy(sizePolicy1)
        self.game_one_bt.setMinimumSize(QSize(150, 50))

        self.verticalLayout_4.addWidget(self.game_one_bt)

        self.game_two_bt = PrimaryPushButton(self.leftWidget)
        self.game_two_bt.setObjectName(u"game_two_bt")
        self.game_two_bt.setMinimumSize(QSize(150, 50))

        self.verticalLayout_4.addWidget(self.game_two_bt)

        self.game_three_bt = PrimaryPushButton(self.leftWidget)
        self.game_three_bt.setObjectName(u"game_three_bt")
        self.game_three_bt.setMinimumSize(QSize(150, 50))

        self.verticalLayout_4.addWidget(self.game_three_bt)


        self.horizontalLayout.addWidget(self.leftWidget, 0, Qt.AlignHCenter)

        self.stackedWidget = QStackedWidget(self.baseWidget)
        self.stackedWidget.setObjectName(u"stackedWidget")
        sizePolicy.setHeightForWidth(self.stackedWidget.sizePolicy().hasHeightForWidth())
        self.stackedWidget.setSizePolicy(sizePolicy)
        self.stackedWidget.setMinimumSize(QSize(0, 0))
        self.stackedWidget.setStyleSheet(u"")
        self.page = QWidget()
        self.page.setObjectName(u"page")
        self.horizontalLayout_2 = QHBoxLayout(self.page)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalSpacer_6 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_6)

        self.chessboardWidget = QWidget(self.page)
        self.chessboardWidget.setObjectName(u"chessboardWidget")
        sizePolicy.setHeightForWidth(self.chessboardWidget.sizePolicy().hasHeightForWidth())
        self.chessboardWidget.setSizePolicy(sizePolicy)
        self.chessboardWidget.setMinimumSize(QSize(680, 540))
        self.chessboardWidget.setStyleSheet(u"")
        self.verticalLayout_10 = QVBoxLayout(self.chessboardWidget)
        self.verticalLayout_10.setSpacing(0)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.verticalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacer_4 = QSpacerItem(20, 1, QSizePolicy.Minimum, QSizePolicy.Fixed)

        self.verticalLayout_10.addItem(self.verticalSpacer_4)

        self.chessboardLabel = ImageLabel(self.chessboardWidget)
        self.chessboardLabel.setObjectName(u"chessboardLabel")
        sizePolicy.setHeightForWidth(self.chessboardLabel.sizePolicy().hasHeightForWidth())
        self.chessboardLabel.setSizePolicy(sizePolicy)
        self.chessboardLabel.setMinimumSize(QSize(680, 540))

        self.verticalLayout_10.addWidget(self.chessboardLabel, 0, Qt.AlignHCenter)

        self.verticalSpacer_5 = QSpacerItem(20, 1, QSizePolicy.Minimum, QSizePolicy.Fixed)

        self.verticalLayout_10.addItem(self.verticalSpacer_5)


        self.horizontalLayout_2.addWidget(self.chessboardWidget)

        self.horizontalSpacer_7 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_7)

        self.gogangWidget = QWidget(self.page)
        self.gogangWidget.setObjectName(u"gogangWidget")
        sizePolicy2 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.gogangWidget.sizePolicy().hasHeightForWidth())
        self.gogangWidget.setSizePolicy(sizePolicy2)
        self.gogangWidget.setMinimumSize(QSize(0, 0))
        self.gogangWidget.setMaximumSize(QSize(16777215, 16777215))
        self.verticalLayout = QVBoxLayout(self.gogangWidget)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacer_9 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer_9)

        self.gobangBtns = QWidget(self.gogangWidget)
        self.gobangBtns.setObjectName(u"gobangBtns")
        self.gobangBtns.setMinimumSize(QSize(0, 0))
        self.verticalLayout_2 = QVBoxLayout(self.gobangBtns)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.startButton = PushButton(self.gobangBtns)
        self.startButton.setObjectName(u"startButton")
        sizePolicy1.setHeightForWidth(self.startButton.sizePolicy().hasHeightForWidth())
        self.startButton.setSizePolicy(sizePolicy1)
        self.startButton.setMinimumSize(QSize(150, 50))

        self.verticalLayout_2.addWidget(self.startButton)

        self.verticalSpacer_13 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer_13)

        self.ruleButton = PushButton(self.gobangBtns)
        self.ruleButton.setObjectName(u"ruleButton")
        self.ruleButton.setMinimumSize(QSize(150, 50))

        self.verticalLayout_2.addWidget(self.ruleButton)

        self.verticalSpacer_14 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer_14)

        self.degreeButton = PushButton(self.gobangBtns)
        self.degreeButton.setObjectName(u"degreeButton")
        self.degreeButton.setMinimumSize(QSize(150, 50))

        self.verticalLayout_2.addWidget(self.degreeButton)

        self.verticalSpacer_15 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer_15)

        self.stopbutton = PushButton(self.gobangBtns)
        self.stopbutton.setObjectName(u"stopbutton")
        self.stopbutton.setMinimumSize(QSize(150, 50))

        self.verticalLayout_2.addWidget(self.stopbutton)


        self.verticalLayout.addWidget(self.gobangBtns, 0, Qt.AlignHCenter)

        self.verticalSpacer = QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)

        self.turnWidget = QWidget(self.gogangWidget)
        self.turnWidget.setObjectName(u"turnWidget")
        self.verticalLayout_3 = QVBoxLayout(self.turnWidget)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.playerLabel = BodyLabel(self.turnWidget)
        self.playerLabel.setObjectName(u"playerLabel")
        self.playerLabel.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.playerLabel)

        self.playerName = BodyLabel(self.turnWidget)
        self.playerName.setObjectName(u"playerName")
        self.playerName.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.playerName)


        self.verticalLayout.addWidget(self.turnWidget, 0, Qt.AlignHCenter)

        self.verticalSpacer_12 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer_12)

        self.verticalLayout.setStretch(0, 1)
        self.verticalLayout.setStretch(1, 7)
        self.verticalLayout.setStretch(2, 1)
        self.verticalLayout.setStretch(3, 2)
        self.verticalLayout.setStretch(4, 1)

        self.horizontalLayout_2.addWidget(self.gogangWidget)

        self.horizontalLayout_2.setStretch(0, 5)
        self.horizontalLayout_2.setStretch(1, 68)
        self.horizontalLayout_2.setStretch(2, 5)
        self.horizontalLayout_2.setStretch(3, 18)
        self.stackedWidget.addWidget(self.page)
        self.page_2 = QWidget()
        self.page_2.setObjectName(u"page_2")
        self.horizontalLayout_3 = QHBoxLayout(self.page_2)
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer)

        self.game2_table = QTableWidget(self.page_2)
        if (self.game2_table.columnCount() < 5):
            self.game2_table.setColumnCount(5)
        __qtablewidgetitem = QTableWidgetItem()
        self.game2_table.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.game2_table.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.game2_table.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.game2_table.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.game2_table.setHorizontalHeaderItem(4, __qtablewidgetitem4)
        if (self.game2_table.rowCount() < 5):
            self.game2_table.setRowCount(5)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.game2_table.setVerticalHeaderItem(0, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.game2_table.setVerticalHeaderItem(1, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.game2_table.setVerticalHeaderItem(2, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.game2_table.setVerticalHeaderItem(3, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        self.game2_table.setVerticalHeaderItem(4, __qtablewidgetitem9)
        self.game2_table.setObjectName(u"game2_table")
        sizePolicy.setHeightForWidth(self.game2_table.sizePolicy().hasHeightForWidth())
        self.game2_table.setSizePolicy(sizePolicy)
        self.game2_table.setMinimumSize(QSize(0, 0))
        self.game2_table.setMaximumSize(QSize(16777215, 16777215))
        self.game2_table.horizontalHeader().setCascadingSectionResizes(False)
        self.game2_table.horizontalHeader().setDefaultSectionSize(100)
        self.game2_table.verticalHeader().setMinimumSectionSize(21)

        self.horizontalLayout_3.addWidget(self.game2_table)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer_2)

        self.rightWidget1 = QWidget(self.page_2)
        self.rightWidget1.setObjectName(u"rightWidget1")
        sizePolicy.setHeightForWidth(self.rightWidget1.sizePolicy().hasHeightForWidth())
        self.rightWidget1.setSizePolicy(sizePolicy)
        self.rightWidget1.setMinimumSize(QSize(0, 0))
        self.rightWidget1.setMaximumSize(QSize(16777215, 16777215))
        self.rightWidget1.setLayoutDirection(Qt.LeftToRight)
        self.rightWidget1.setAutoFillBackground(False)
        self.verticalLayout_7 = QVBoxLayout(self.rightWidget1)
        self.verticalLayout_7.setSpacing(0)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacer_16 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_7.addItem(self.verticalSpacer_16)

        self.stackBtns = QWidget(self.rightWidget1)
        self.stackBtns.setObjectName(u"stackBtns")
        sizePolicy3 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.stackBtns.sizePolicy().hasHeightForWidth())
        self.stackBtns.setSizePolicy(sizePolicy3)
        self.stackBtns.setMinimumSize(QSize(0, 0))
        self.stackBtns.setMaximumSize(QSize(16777215, 16777215))
        self.verticalLayout_6 = QVBoxLayout(self.stackBtns)
        self.verticalLayout_6.setSpacing(0)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.game2_setButton = PushButton(self.stackBtns)
        self.game2_setButton.setObjectName(u"game2_setButton")
        sizePolicy1.setHeightForWidth(self.game2_setButton.sizePolicy().hasHeightForWidth())
        self.game2_setButton.setSizePolicy(sizePolicy1)
        self.game2_setButton.setMinimumSize(QSize(150, 50))

        self.verticalLayout_6.addWidget(self.game2_setButton)

        self.verticalSpacer_17 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_6.addItem(self.verticalSpacer_17)

        self.game2_setButton_2 = PushButton(self.stackBtns)
        self.game2_setButton_2.setObjectName(u"game2_setButton_2")
        self.game2_setButton_2.setMinimumSize(QSize(150, 50))

        self.verticalLayout_6.addWidget(self.game2_setButton_2)

        self.verticalSpacer_18 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_6.addItem(self.verticalSpacer_18)

        self.game2_setButton_3 = PushButton(self.stackBtns)
        self.game2_setButton_3.setObjectName(u"game2_setButton_3")
        self.game2_setButton_3.setMinimumSize(QSize(150, 50))

        self.verticalLayout_6.addWidget(self.game2_setButton_3)


        self.verticalLayout_7.addWidget(self.stackBtns, 0, Qt.AlignHCenter)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_7.addItem(self.verticalSpacer_2)

        self.label_2 = BodyLabel(self.rightWidget1)
        self.label_2.setObjectName(u"label_2")
        sizePolicy3.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy3)
        self.label_2.setMinimumSize(QSize(60, 40))
        self.label_2.setTextFormat(Qt.AutoText)
        self.label_2.setAlignment(Qt.AlignCenter)

        self.verticalLayout_7.addWidget(self.label_2, 0, Qt.AlignHCenter)

        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setSpacing(10)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.gridLayout_3.setSizeConstraint(QLayout.SetMinimumSize)
        self.game2_put_Button_4 = PushButton(self.rightWidget1)
        self.game2_put_Button_4.setObjectName(u"game2_put_Button_4")
        sizePolicy3.setHeightForWidth(self.game2_put_Button_4.sizePolicy().hasHeightForWidth())
        self.game2_put_Button_4.setSizePolicy(sizePolicy3)
        self.game2_put_Button_4.setMinimumSize(QSize(70, 40))

        self.gridLayout_3.addWidget(self.game2_put_Button_4, 3, 1, 1, 1)

        self.game2_put_Button_6 = PushButton(self.rightWidget1)
        self.game2_put_Button_6.setObjectName(u"game2_put_Button_6")
        sizePolicy3.setHeightForWidth(self.game2_put_Button_6.sizePolicy().hasHeightForWidth())
        self.game2_put_Button_6.setSizePolicy(sizePolicy3)
        self.game2_put_Button_6.setMinimumSize(QSize(70, 40))

        self.gridLayout_3.addWidget(self.game2_put_Button_6, 4, 1, 1, 1)

        self.game2_put_Button_2 = PushButton(self.rightWidget1)
        self.game2_put_Button_2.setObjectName(u"game2_put_Button_2")
        sizePolicy3.setHeightForWidth(self.game2_put_Button_2.sizePolicy().hasHeightForWidth())
        self.game2_put_Button_2.setSizePolicy(sizePolicy3)
        self.game2_put_Button_2.setMinimumSize(QSize(70, 40))

        self.gridLayout_3.addWidget(self.game2_put_Button_2, 2, 1, 1, 1)

        self.game2_put_Button_3 = PushButton(self.rightWidget1)
        self.game2_put_Button_3.setObjectName(u"game2_put_Button_3")
        sizePolicy3.setHeightForWidth(self.game2_put_Button_3.sizePolicy().hasHeightForWidth())
        self.game2_put_Button_3.setSizePolicy(sizePolicy3)
        self.game2_put_Button_3.setMinimumSize(QSize(70, 40))

        self.gridLayout_3.addWidget(self.game2_put_Button_3, 3, 0, 1, 1)

        self.game2_put_Button_1 = PushButton(self.rightWidget1)
        self.game2_put_Button_1.setObjectName(u"game2_put_Button_1")
        sizePolicy3.setHeightForWidth(self.game2_put_Button_1.sizePolicy().hasHeightForWidth())
        self.game2_put_Button_1.setSizePolicy(sizePolicy3)
        self.game2_put_Button_1.setMinimumSize(QSize(70, 40))

        self.gridLayout_3.addWidget(self.game2_put_Button_1, 2, 0, 1, 1)

        self.game2_put_Button_5 = PushButton(self.rightWidget1)
        self.game2_put_Button_5.setObjectName(u"game2_put_Button_5")
        sizePolicy3.setHeightForWidth(self.game2_put_Button_5.sizePolicy().hasHeightForWidth())
        self.game2_put_Button_5.setSizePolicy(sizePolicy3)
        self.game2_put_Button_5.setMinimumSize(QSize(70, 40))

        self.gridLayout_3.addWidget(self.game2_put_Button_5, 4, 0, 1, 1)


        self.verticalLayout_7.addLayout(self.gridLayout_3)

        self.verticalSpacer_10 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_7.addItem(self.verticalSpacer_10)


        self.horizontalLayout_3.addWidget(self.rightWidget1, 0, Qt.AlignHCenter)

        self.horizontalLayout_3.setStretch(0, 2)
        self.horizontalLayout_3.setStretch(1, 9)
        self.horizontalLayout_3.setStretch(2, 2)
        self.horizontalLayout_3.setStretch(3, 3)
        self.stackedWidget.addWidget(self.page_2)
        self.page_3 = QWidget()
        self.page_3.setObjectName(u"page_3")
        self.horizontalLayout_4 = QHBoxLayout(self.page_3)
        self.horizontalLayout_4.setSpacing(0)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalSpacer_8 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_8)

        self.game3_table = QTableWidget(self.page_3)
        if (self.game3_table.columnCount() < 5):
            self.game3_table.setColumnCount(5)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.game3_table.setHorizontalHeaderItem(0, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        self.game3_table.setHorizontalHeaderItem(1, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        self.game3_table.setHorizontalHeaderItem(2, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        self.game3_table.setHorizontalHeaderItem(3, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        self.game3_table.setHorizontalHeaderItem(4, __qtablewidgetitem14)
        if (self.game3_table.rowCount() < 5):
            self.game3_table.setRowCount(5)
        __qtablewidgetitem15 = QTableWidgetItem()
        self.game3_table.setVerticalHeaderItem(0, __qtablewidgetitem15)
        __qtablewidgetitem16 = QTableWidgetItem()
        self.game3_table.setVerticalHeaderItem(1, __qtablewidgetitem16)
        __qtablewidgetitem17 = QTableWidgetItem()
        self.game3_table.setVerticalHeaderItem(2, __qtablewidgetitem17)
        __qtablewidgetitem18 = QTableWidgetItem()
        self.game3_table.setVerticalHeaderItem(3, __qtablewidgetitem18)
        __qtablewidgetitem19 = QTableWidgetItem()
        self.game3_table.setVerticalHeaderItem(4, __qtablewidgetitem19)
        self.game3_table.setObjectName(u"game3_table")
        sizePolicy.setHeightForWidth(self.game3_table.sizePolicy().hasHeightForWidth())
        self.game3_table.setSizePolicy(sizePolicy)
        self.game3_table.setMinimumSize(QSize(0, 0))
        self.game3_table.setMaximumSize(QSize(16777215, 16777215))
        self.game3_table.setShowGrid(True)
        self.game3_table.setGridStyle(Qt.SolidLine)
        self.game3_table.setSortingEnabled(False)
        self.game3_table.horizontalHeader().setCascadingSectionResizes(False)
        self.game3_table.horizontalHeader().setProperty("showSortIndicator", False)
        self.game3_table.horizontalHeader().setStretchLastSection(False)
        self.game3_table.verticalHeader().setCascadingSectionResizes(False)
        self.game3_table.verticalHeader().setHighlightSections(True)
        self.game3_table.verticalHeader().setProperty("showSortIndicator", False)
        self.game3_table.verticalHeader().setStretchLastSection(False)

        self.horizontalLayout_4.addWidget(self.game3_table)

        self.horizontalSpacer_9 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_9)

        self.rightWidget2 = QWidget(self.page_3)
        self.rightWidget2.setObjectName(u"rightWidget2")
        sizePolicy.setHeightForWidth(self.rightWidget2.sizePolicy().hasHeightForWidth())
        self.rightWidget2.setSizePolicy(sizePolicy)
        self.rightWidget2.setMaximumSize(QSize(250, 16777215))
        self.verticalLayout_9 = QVBoxLayout(self.rightWidget2)
        self.verticalLayout_9.setSpacing(0)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.verticalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacer_19 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_9.addItem(self.verticalSpacer_19)

        self.putBtns = QWidget(self.rightWidget2)
        self.putBtns.setObjectName(u"putBtns")
        self.putBtns.setMinimumSize(QSize(0, 0))
        self.putBtns.setMaximumSize(QSize(16777215, 16777215))
        self.verticalLayout_8 = QVBoxLayout(self.putBtns)
        self.verticalLayout_8.setSpacing(0)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.verticalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.game3_setButton = PushButton(self.putBtns)
        self.game3_setButton.setObjectName(u"game3_setButton")
        self.game3_setButton.setMinimumSize(QSize(150, 50))

        self.verticalLayout_8.addWidget(self.game3_setButton)

        self.verticalSpacer_20 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_8.addItem(self.verticalSpacer_20)

        self.game3_setButton_2 = PushButton(self.putBtns)
        self.game3_setButton_2.setObjectName(u"game3_setButton_2")
        self.game3_setButton_2.setMinimumSize(QSize(150, 50))

        self.verticalLayout_8.addWidget(self.game3_setButton_2)

        self.verticalSpacer_21 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_8.addItem(self.verticalSpacer_21)

        self.game3_setButton_3 = PushButton(self.putBtns)
        self.game3_setButton_3.setObjectName(u"game3_setButton_3")
        self.game3_setButton_3.setMinimumSize(QSize(150, 50))

        self.verticalLayout_8.addWidget(self.game3_setButton_3)


        self.verticalLayout_9.addWidget(self.putBtns, 0, Qt.AlignHCenter)

        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_9.addItem(self.verticalSpacer_3)

        self.label_3 = BodyLabel(self.rightWidget2)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setMinimumSize(QSize(60, 40))
        self.label_3.setFrameShadow(QFrame.Plain)
        self.label_3.setAlignment(Qt.AlignCenter)

        self.verticalLayout_9.addWidget(self.label_3, 0, Qt.AlignHCenter)

        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setSpacing(10)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.game3_numButton_2 = PushButton(self.rightWidget2)
        self.game3_numButton_2.setObjectName(u"game3_numButton_2")
        sizePolicy3.setHeightForWidth(self.game3_numButton_2.sizePolicy().hasHeightForWidth())
        self.game3_numButton_2.setSizePolicy(sizePolicy3)
        self.game3_numButton_2.setMinimumSize(QSize(70, 40))

        self.gridLayout_2.addWidget(self.game3_numButton_2, 1, 1, 1, 1)

        self.game3_numButton_4 = PushButton(self.rightWidget2)
        self.game3_numButton_4.setObjectName(u"game3_numButton_4")
        sizePolicy3.setHeightForWidth(self.game3_numButton_4.sizePolicy().hasHeightForWidth())
        self.game3_numButton_4.setSizePolicy(sizePolicy3)
        self.game3_numButton_4.setMinimumSize(QSize(70, 40))

        self.gridLayout_2.addWidget(self.game3_numButton_4, 2, 1, 1, 1)

        self.game3_numButton_3 = PushButton(self.rightWidget2)
        self.game3_numButton_3.setObjectName(u"game3_numButton_3")
        sizePolicy3.setHeightForWidth(self.game3_numButton_3.sizePolicy().hasHeightForWidth())
        self.game3_numButton_3.setSizePolicy(sizePolicy3)
        self.game3_numButton_3.setMinimumSize(QSize(70, 40))

        self.gridLayout_2.addWidget(self.game3_numButton_3, 2, 0, 1, 1)

        self.game3_numButton_5 = PushButton(self.rightWidget2)
        self.game3_numButton_5.setObjectName(u"game3_numButton_5")
        sizePolicy3.setHeightForWidth(self.game3_numButton_5.sizePolicy().hasHeightForWidth())
        self.game3_numButton_5.setSizePolicy(sizePolicy3)
        self.game3_numButton_5.setMinimumSize(QSize(70, 40))

        self.gridLayout_2.addWidget(self.game3_numButton_5, 4, 0, 1, 1)

        self.game3_numButton_1 = PushButton(self.rightWidget2)
        self.game3_numButton_1.setObjectName(u"game3_numButton_1")
        sizePolicy3.setHeightForWidth(self.game3_numButton_1.sizePolicy().hasHeightForWidth())
        self.game3_numButton_1.setSizePolicy(sizePolicy3)
        self.game3_numButton_1.setMinimumSize(QSize(70, 40))

        self.gridLayout_2.addWidget(self.game3_numButton_1, 1, 0, 1, 1)

        self.game3_numButton_6 = PushButton(self.rightWidget2)
        self.game3_numButton_6.setObjectName(u"game3_numButton_6")
        sizePolicy3.setHeightForWidth(self.game3_numButton_6.sizePolicy().hasHeightForWidth())
        self.game3_numButton_6.setSizePolicy(sizePolicy3)
        self.game3_numButton_6.setMinimumSize(QSize(70, 40))

        self.gridLayout_2.addWidget(self.game3_numButton_6, 4, 1, 1, 1)


        self.verticalLayout_9.addLayout(self.gridLayout_2)

        self.verticalSpacer_11 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_9.addItem(self.verticalSpacer_11)


        self.horizontalLayout_4.addWidget(self.rightWidget2, 0, Qt.AlignHCenter)

        self.horizontalLayout_4.setStretch(0, 2)
        self.horizontalLayout_4.setStretch(1, 9)
        self.horizontalLayout_4.setStretch(2, 2)
        self.horizontalLayout_4.setStretch(3, 3)
        self.stackedWidget.addWidget(self.page_3)

        self.horizontalLayout.addWidget(self.stackedWidget)

        self.horizontalSpacer_5 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_5)

        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 4)
        self.horizontalLayout.setStretch(2, 18)
        self.horizontalLayout.setStretch(3, 1)

        self.verticalLayout_17.addWidget(self.baseWidget)

        self.verticalSpacer_8 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Minimum)

        self.verticalLayout_17.addItem(self.verticalSpacer_8)

        self.verticalLayout_17.setStretch(0, 1)
        self.verticalLayout_17.setStretch(1, 6)
        self.verticalLayout_17.setStretch(2, 1)

        self.verticalLayout_5.addWidget(self.mainWidget)


        self.retranslateUi(Form)

        self.stackedWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.iconlabel.setText("")
        self.game_one_bt.setText(QCoreApplication.translate("Form", u"\u4e94\u5b50\u68cb\u6e38\u620f", None))
        self.game_two_bt.setText(QCoreApplication.translate("Form", u"\u5806\u53e0\u65b9\u5757", None))
        self.game_three_bt.setText(QCoreApplication.translate("Form", u"\u6446\u653e\u65b9\u5757", None))
        self.chessboardLabel.setText(QCoreApplication.translate("Form", u"1", None))
        self.startButton.setText(QCoreApplication.translate("Form", u"\u5f00\u59cb\u6e38\u620f", None))
        self.ruleButton.setText(QCoreApplication.translate("Form", u"\u6e38\u620f\u89c4\u5219", None))
        self.degreeButton.setText(QCoreApplication.translate("Form", u"\u96be\u5ea6\u8bbe\u7f6e", None))
        self.stopbutton.setText(QCoreApplication.translate("Form", u"\u7ed3\u675f\u6e38\u620f", None))
        self.playerLabel.setText(QCoreApplication.translate("Form", u"<html><head/><body><p><span style=\" font-size:20pt; font-weight:600;\">Player</span></p></body></html>", None))
        self.playerName.setText(QCoreApplication.translate("Form", u"<html><head/><body><p><span style=\" font-size:22pt; font-weight:600; color:#ef2929;\">Human</span></p></body></html>", None))
        ___qtablewidgetitem = self.game2_table.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem1 = self.game2_table.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem2 = self.game2_table.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem3 = self.game2_table.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem4 = self.game2_table.horizontalHeaderItem(4)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem5 = self.game2_table.verticalHeaderItem(0)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem6 = self.game2_table.verticalHeaderItem(1)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem7 = self.game2_table.verticalHeaderItem(2)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem8 = self.game2_table.verticalHeaderItem(3)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem9 = self.game2_table.verticalHeaderItem(4)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        self.game2_setButton.setText(QCoreApplication.translate("Form", u"\u8bbe\u7f6e\u5806\u53e0\u6837\u5f0f", None))
        self.game2_setButton_2.setText(QCoreApplication.translate("Form", u"\u5f00\u59cb\u5806\u53e0", None))
        self.game2_setButton_3.setText(QCoreApplication.translate("Form", u"\u6e05\u7a7a\u6837\u5f0f", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"<html><head/><body><p align=\"center\"><span style=\" font-size:16pt;\">\u6837\u5f0f\u6a21\u7248</span></p></body></html>", None))
        self.game2_put_Button_4.setText(QCoreApplication.translate("Form", u"\u6a21\u72484", None))
        self.game2_put_Button_6.setText(QCoreApplication.translate("Form", u"\u6a21\u72486", None))
        self.game2_put_Button_2.setText(QCoreApplication.translate("Form", u"\u6a21\u72482", None))
        self.game2_put_Button_3.setText(QCoreApplication.translate("Form", u"\u6a21\u72483", None))
        self.game2_put_Button_1.setText(QCoreApplication.translate("Form", u"\u6a21\u72481", None))
        self.game2_put_Button_5.setText(QCoreApplication.translate("Form", u"\u6a21\u72485", None))
        ___qtablewidgetitem10 = self.game3_table.horizontalHeaderItem(0)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem11 = self.game3_table.horizontalHeaderItem(1)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem12 = self.game3_table.horizontalHeaderItem(2)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem13 = self.game3_table.horizontalHeaderItem(3)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem14 = self.game3_table.horizontalHeaderItem(4)
        ___qtablewidgetitem14.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u5217", None));
        ___qtablewidgetitem15 = self.game3_table.verticalHeaderItem(0)
        ___qtablewidgetitem15.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem16 = self.game3_table.verticalHeaderItem(1)
        ___qtablewidgetitem16.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem17 = self.game3_table.verticalHeaderItem(2)
        ___qtablewidgetitem17.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem18 = self.game3_table.verticalHeaderItem(3)
        ___qtablewidgetitem18.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        ___qtablewidgetitem19 = self.game3_table.verticalHeaderItem(4)
        ___qtablewidgetitem19.setText(QCoreApplication.translate("Form", u"\u65b0\u5efa\u884c", None));
        self.game3_setButton.setText(QCoreApplication.translate("Form", u"\u8bbe\u7f6e\u6446\u653e\u6837\u5f0f", None))
        self.game3_setButton_2.setText(QCoreApplication.translate("Form", u"\u5f00\u59cb\u6446\u653e", None))
        self.game3_setButton_3.setText(QCoreApplication.translate("Form", u"\u6e05\u7a7a\u6837\u5f0f", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"<html><head/><body><p align=\"center\"><span style=\" font-size:16pt;\">\u6837\u5f0f\u6a21\u7248</span></p></body></html>", None))
        self.game3_numButton_2.setText(QCoreApplication.translate("Form", u"\u6570\u5b576", None))
        self.game3_numButton_4.setText(QCoreApplication.translate("Form", u"\u5b57\u6bcdP", None))
        self.game3_numButton_3.setText(QCoreApplication.translate("Form", u"\u6570\u5b578", None))
        self.game3_numButton_5.setText(QCoreApplication.translate("Form", u"\u5b57\u6bcdX", None))
        self.game3_numButton_1.setText(QCoreApplication.translate("Form", u"\u6570\u5b574", None))
        self.game3_numButton_6.setText(QCoreApplication.translate("Form", u"\u5370\u82b1", None))
    # retranslateUi

