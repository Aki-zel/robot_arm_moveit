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
from qfluentwidgets import PrimaryPushButton


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(852, 704)
        self.stackedWidget = QStackedWidget(Form)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.stackedWidget.setGeometry(QRect(10, 50, 881, 641))
        self.page = QWidget()
        self.page.setObjectName(u"page")
        self.label = BodyLabel(self.page)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(670, 220, 71, 41))
        self.imageLabel = ImageLabel(self.page)
        self.imageLabel.setObjectName(u"imageLabel")
        self.imageLabel.setGeometry(QRect(10, 30, 600, 600))
        self.splitter_2 = QSplitter(self.page)
        self.splitter_2.setObjectName(u"splitter_2")
        self.splitter_2.setGeometry(QRect(660, 30, 121, 101))
        self.splitter_2.setOrientation(Qt.Vertical)
        self.startButton = PushButton(self.splitter_2)
        self.startButton.setObjectName(u"startButton")
        self.splitter_2.addWidget(self.startButton)
        self.ruleButton = TransparentPushButton(self.splitter_2)
        self.ruleButton.setObjectName(u"ruleButton")
        self.splitter_2.addWidget(self.ruleButton)
        self.playerName = BodyLabel(self.page)
        self.playerName.setObjectName(u"playerName")
        self.playerName.setGeometry(QRect(670, 260, 101, 41))
        self.stopbutton = PushButton(self.page)
        self.stopbutton.setObjectName(u"stopbutton")
        self.stopbutton.setGeometry(QRect(660, 140, 121, 49))
        self.stackedWidget.addWidget(self.page)
        self.page_2 = QWidget()
        self.page_2.setObjectName(u"page_2")
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
        self.game2_table.setGeometry(QRect(20, 70, 500, 500))
        self.game2_setButton = PushButton(self.page_2)
        self.game2_setButton.setObjectName(u"game2_setButton")
        self.game2_setButton.setGeometry(QRect(620, 70, 111, 41))
        self.game2_setButton_2 = PushButton(self.page_2)
        self.game2_setButton_2.setObjectName(u"game2_setButton_2")
        self.game2_setButton_2.setGeometry(QRect(620, 130, 111, 41))
        self.game2_imageBox = QLabel(self.page_2)
        self.game2_imageBox.setObjectName(u"game2_imageBox")
        self.game2_imageBox.setGeometry(QRect(550, 350, 250, 250))
        self.game2_setButton_3 = PushButton(self.page_2)
        self.game2_setButton_3.setObjectName(u"game2_setButton_3")
        self.game2_setButton_3.setGeometry(QRect(620, 190, 111, 41))
        self.widget = QWidget(self.page_2)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(540, 240, 281, 101))
        self.gridLayout_3 = QGridLayout(self.widget)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.label_2 = BodyLabel(self.widget)
        self.label_2.setObjectName(u"label_2")

        self.gridLayout_3.addWidget(self.label_2, 0, 0, 1, 1)

        self.game2_put_Button_1 = PushButton(self.widget)
        self.game2_put_Button_1.setObjectName(u"game2_put_Button_1")

        self.gridLayout_3.addWidget(self.game2_put_Button_1, 1, 0, 1, 1)

        self.game2_put_Button_2 = PushButton(self.widget)
        self.game2_put_Button_2.setObjectName(u"game2_put_Button_2")

        self.gridLayout_3.addWidget(self.game2_put_Button_2, 1, 1, 1, 1)

        self.game2_put_Button_3 = PushButton(self.widget)
        self.game2_put_Button_3.setObjectName(u"game2_put_Button_3")

        self.gridLayout_3.addWidget(self.game2_put_Button_3, 1, 2, 1, 1)

        self.game2_put_Button_4 = PushButton(self.widget)
        self.game2_put_Button_4.setObjectName(u"game2_put_Button_4")

        self.gridLayout_3.addWidget(self.game2_put_Button_4, 2, 0, 1, 1)

        self.game2_put_Button_5 = PushButton(self.widget)
        self.game2_put_Button_5.setObjectName(u"game2_put_Button_5")

        self.gridLayout_3.addWidget(self.game2_put_Button_5, 2, 1, 1, 1)

        self.game2_put_Button_6 = PushButton(self.widget)
        self.game2_put_Button_6.setObjectName(u"game2_put_Button_6")

        self.gridLayout_3.addWidget(self.game2_put_Button_6, 2, 2, 1, 1)

        self.stackedWidget.addWidget(self.page_2)
        self.page_3 = QWidget()
        self.page_3.setObjectName(u"page_3")
        self.game3_setButton_2 = PushButton(self.page_3)
        self.game3_setButton_2.setObjectName(u"game3_setButton_2")
        self.game3_setButton_2.setGeometry(QRect(620, 130, 111, 41))
        self.game3_imageBox = QLabel(self.page_3)
        self.game3_imageBox.setObjectName(u"game3_imageBox")
        self.game3_imageBox.setGeometry(QRect(560, 360, 250, 250))
        self.game3_setButton = PushButton(self.page_3)
        self.game3_setButton.setObjectName(u"game3_setButton")
        self.game3_setButton.setGeometry(QRect(620, 70, 111, 41))
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
        self.game3_table.setGeometry(QRect(20, 70, 500, 500))
        self.game3_setButton_3 = PushButton(self.page_3)
        self.game3_setButton_3.setObjectName(u"game3_setButton_3")
        self.game3_setButton_3.setGeometry(QRect(620, 190, 111, 41))
        self.layoutWidget_2 = QWidget(self.page_3)
        self.layoutWidget_2.setObjectName(u"layoutWidget_2")
        self.layoutWidget_2.setGeometry(QRect(540, 240, 281, 111))
        self.gridLayout_2 = QGridLayout(self.layoutWidget_2)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.game3_numButton_2 = PushButton(self.layoutWidget_2)
        self.game3_numButton_2.setObjectName(u"game3_numButton_2")

        self.gridLayout_2.addWidget(self.game3_numButton_2, 1, 1, 1, 1)

        self.game3_numButton_1 = PushButton(self.layoutWidget_2)
        self.game3_numButton_1.setObjectName(u"game3_numButton_1")

        self.gridLayout_2.addWidget(self.game3_numButton_1, 1, 0, 1, 1)

        self.game3_numButton_5 = PushButton(self.layoutWidget_2)
        self.game3_numButton_5.setObjectName(u"game3_numButton_5")

        self.gridLayout_2.addWidget(self.game3_numButton_5, 3, 1, 1, 1)

        self.game3_numButton_4 = PushButton(self.layoutWidget_2)
        self.game3_numButton_4.setObjectName(u"game3_numButton_4")

        self.gridLayout_2.addWidget(self.game3_numButton_4, 3, 0, 1, 1)

        self.game3_numButton_3 = PushButton(self.layoutWidget_2)
        self.game3_numButton_3.setObjectName(u"game3_numButton_3")

        self.gridLayout_2.addWidget(self.game3_numButton_3, 1, 2, 1, 1)

        self.game3_numButton_6 = PushButton(self.layoutWidget_2)
        self.game3_numButton_6.setObjectName(u"game3_numButton_6")

        self.gridLayout_2.addWidget(self.game3_numButton_6, 3, 2, 1, 1)

        self.label_3 = BodyLabel(self.layoutWidget_2)
        self.label_3.setObjectName(u"label_3")

        self.gridLayout_2.addWidget(self.label_3, 0, 0, 1, 1)

        self.stackedWidget.addWidget(self.page_3)
        self.layoutWidget = QWidget(Form)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(10, 10, 591, 61))
        self.horizontalLayout = QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.game_one_bt = PrimaryPushButton(self.layoutWidget)
        self.game_one_bt.setObjectName(u"game_one_bt")

        self.horizontalLayout.addWidget(self.game_one_bt)

        self.game_two_bt = PrimaryPushButton(self.layoutWidget)
        self.game_two_bt.setObjectName(u"game_two_bt")

        self.horizontalLayout.addWidget(self.game_two_bt)

        self.game_three_bt = PrimaryPushButton(self.layoutWidget)
        self.game_three_bt.setObjectName(u"game_three_bt")

        self.horizontalLayout.addWidget(self.game_three_bt)


        self.retranslateUi(Form)

        self.stackedWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label.setText(QCoreApplication.translate("Form", u"<html><head/><body><p><span style=\" font-size:16pt; font-weight:600;\">Player</span></p></body></html>", None))
        self.imageLabel.setText(QCoreApplication.translate("Form", u"1", None))
        self.startButton.setText(QCoreApplication.translate("Form", u"\u5f00\u59cb\u6e38\u620f", None))
        self.ruleButton.setText(QCoreApplication.translate("Form", u"\u6e38\u620f\u89c4\u5219", None))
        self.playerName.setText(QCoreApplication.translate("Form", u"<html><head/><body><p><span style=\" font-size:22pt; font-weight:600; color:#ef2929;\">Human</span></p></body></html>", None))
        self.stopbutton.setText(QCoreApplication.translate("Form", u"\u7ed3\u675f\u6e38\u620f", None))
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
        self.game2_imageBox.setText("")
        self.game2_setButton_3.setText(QCoreApplication.translate("Form", u"\u4e00\u952e\u6e05\u7a7a", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"\u6a21\u7248\uff1a", None))
        self.game2_put_Button_1.setText(QCoreApplication.translate("Form", u"\u6a21\u72481", None))
        self.game2_put_Button_2.setText(QCoreApplication.translate("Form", u"\u6a21\u72482", None))
        self.game2_put_Button_3.setText(QCoreApplication.translate("Form", u"\u6a21\u72483", None))
        self.game2_put_Button_4.setText(QCoreApplication.translate("Form", u"\u6a21\u72484", None))
        self.game2_put_Button_5.setText(QCoreApplication.translate("Form", u"\u6a21\u72485", None))
        self.game2_put_Button_6.setText(QCoreApplication.translate("Form", u"\u6a21\u72486", None))
        self.game3_setButton_2.setText(QCoreApplication.translate("Form", u"\u5f00\u59cb\u6446\u653e", None))
        self.game3_imageBox.setText("")
        self.game3_setButton.setText(QCoreApplication.translate("Form", u"\u8bbe\u7f6e\u6446\u653e\u6837\u5f0f", None))
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
        self.game3_setButton_3.setText(QCoreApplication.translate("Form", u"\u4e00\u952e\u6e05\u7a7a", None))
        self.game3_numButton_2.setText(QCoreApplication.translate("Form", u"\u6570\u5b576", None))
        self.game3_numButton_1.setText(QCoreApplication.translate("Form", u"\u6570\u5b574", None))
        self.game3_numButton_5.setText(QCoreApplication.translate("Form", u"\u5b57\u7b26X", None))
        self.game3_numButton_4.setText(QCoreApplication.translate("Form", u"\u5b57\u6bcdP", None))
        self.game3_numButton_3.setText(QCoreApplication.translate("Form", u"\u6570\u5b578", None))
        self.game3_numButton_6.setText(QCoreApplication.translate("Form", u"\u5370\u82b1", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"\u6a21\u7248\uff1a", None))
        self.game_one_bt.setText(QCoreApplication.translate("Form", u"\u4e94\u5b50\u68cb\u6e38\u620f", None))
        self.game_two_bt.setText(QCoreApplication.translate("Form", u"\u5806\u53e0\u65b9\u5757", None))
        self.game_three_bt.setText(QCoreApplication.translate("Form", u"\u6446\u653e\u65b9\u5757", None))
    # retranslateUi

