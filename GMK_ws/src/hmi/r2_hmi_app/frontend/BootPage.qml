// BootPage.qml - 阵营选择页
// 专业触控终端设计 - ROBOCON 2026

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: bootPage

    signal teamSelected(int color)

    Rectangle {
        anchors.fill: parent
        color: "#0d1117"

        // 背景装饰线
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            y: parent.height * 0.22
            width: parent.width * 0.7; height: 1
            color: "#30363d"
        }

        // 标题区
        ColumnLayout {
            anchors.top: parent.top
            anchors.topMargin: 36
            anchors.horizontalCenter: parent.horizontalCenter
            spacing: 6

            Text {
                text: "ABU ROBOCON 2026"
                color: "#8b949e"
                font.pixelSize: 15; font.bold: true; font.family: "Consolas"
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: "R2 战术上位机"
                color: "#58a6ff"
                font.pixelSize: 34; font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: "选择比赛阵营以开始"
                color: "#c9d1d9"
                font.pixelSize: 17
                Layout.alignment: Qt.AlignHCenter
            }
        }

        // 选择按钮区
        RowLayout {
            anchors.centerIn: parent
            anchors.verticalCenterOffset: 10
            spacing: 48

            // 红方
            Rectangle {
                id: redCard
                width: 260; height: 310; radius: 18
                color: redArea.pressed ? "#4d1515" : "#1a0a0a"
                border.color: redArea.pressed ? "#ff6b6b" : "#da3633"
                border.width: redArea.pressed ? 4.5 : 2.5

                scale: redArea.pressed ? 0.90 : 1.0
                Behavior on scale { NumberAnimation { duration: 120; easing.type: Easing.OutBack } }
                Behavior on color { ColorAnimation { duration: 100 } }
                Behavior on border.color { ColorAnimation { duration: 100 } }

                // 按压闪光层
                Rectangle {
                    anchors.fill: parent; radius: 18
                    color: "#ffffff"
                    opacity: redArea.pressed ? 0.08 : 0
                    Behavior on opacity { NumberAnimation { duration: 60 } }
                }

                TapHandler {
                    id: redArea
                    onTapped: bootPage.teamSelected(0)
                }

                ColumnLayout {
                    anchors.centerIn: parent
                    spacing: 18

                    Rectangle {
                        width: 110; height: 110; radius: 55
                        color: "#da3633"
                        border.color: "#f85149"; border.width: 3
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            anchors.centerIn: parent
                            text: "R"
                            color: "#fff"; font.pixelSize: 52; font.bold: true; font.family: "Consolas"
                        }
                    }

                    Text {
                        text: "红 方"
                        color: "#f85149"
                        font.pixelSize: 30; font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                    Text {
                        text: "RED TEAM"
                        color: "#f85149"; font.pixelSize: 13; font.family: "Consolas"
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }

            // 蓝方
            Rectangle {
                id: blueCard
                width: 260; height: 310; radius: 18
                color: blueArea.pressed ? "#152545" : "#0a0a1a"
                border.color: blueArea.pressed ? "#79c0ff" : "#1f6feb"
                border.width: blueArea.pressed ? 4.5 : 2.5

                scale: blueArea.pressed ? 0.90 : 1.0
                Behavior on scale { NumberAnimation { duration: 120; easing.type: Easing.OutBack } }
                Behavior on color { ColorAnimation { duration: 100 } }
                Behavior on border.color { ColorAnimation { duration: 100 } }

                // 按压闪光层
                Rectangle {
                    anchors.fill: parent; radius: 18
                    color: "#ffffff"
                    opacity: blueArea.pressed ? 0.08 : 0
                    Behavior on opacity { NumberAnimation { duration: 60 } }
                }

                TapHandler {
                    id: blueArea
                    onTapped: bootPage.teamSelected(1)
                }

                ColumnLayout {
                    anchors.centerIn: parent
                    spacing: 18

                    Rectangle {
                        width: 110; height: 110; radius: 55
                        color: "#1f6feb"
                        border.color: "#58a6ff"; border.width: 3
                        Layout.alignment: Qt.AlignHCenter

                        Text {
                            anchors.centerIn: parent
                            text: "B"
                            color: "#fff"; font.pixelSize: 52; font.bold: true; font.family: "Consolas"
                        }
                    }

                    Text {
                        text: "蓝 方"
                        color: "#58a6ff"
                        font.pixelSize: 30; font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                    Text {
                        text: "BLUE TEAM"
                        color: "#58a6ff"; font.pixelSize: 13; font.family: "Consolas"
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }
        }

        // 底部提示
        Text {
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 30
            anchors.horizontalCenter: parent.horizontalCenter
            text: "7~9\" 触控屏操作 | 无键盘鼠标设计"
            color: "#484f58"
            font.pixelSize: 11; font.family: "Consolas"
        }
    }
}