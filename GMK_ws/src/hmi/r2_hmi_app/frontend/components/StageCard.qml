// StageCard.qml - 赛段进度卡片
// 纵向布局：图标 + 标题 + 状态 + 反馈 + 执行按钮
// 对抗区 (stageId=3) 执行后展示 3×3 九宫格

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: stageCard

    property string title: ""
    property string subtitle: ""
    property string icon: ""
    property int stageId: 0
    property color accentColor: "#58a6ff"
    property string feedbackText: ""
    property bool estopActive: false
    property bool timerRunning: false
    property bool isExecuting: false
    property bool showCombatGrid: false

    signal executeClicked(int stageId)
    signal calibrateClicked(int stageId)

    color: "#161b22"
    radius: 16
    border.color: isExecuting || showCombatGrid ? stageCard.accentColor : "#30363d"
    border.width: 2

    // 普通卡片内容
    ColumnLayout {
        id: normalContent
        visible: !stageCard.showCombatGrid
        anchors.fill: parent
        anchors.margins: 16
        spacing: 10

        // 图标
        Rectangle {
            width: 60; height: 60; radius: 14
            color: stageCard.accentColor
            Layout.alignment: Qt.AlignHCenter
            Text {
                anchors.centerIn: parent
                text: stageCard.icon
                color: "#fff"; font.pixelSize: 28; font.bold: true
            }
        }

        // 标题
        Text {
            text: stageCard.title
            color: "#c9d1d9"
            font.pixelSize: 24; font.bold: true
            Layout.alignment: Qt.AlignHCenter
        }
        Text {
            text: stageCard.subtitle
            color: "#8b949e"
            font.pixelSize: 11; font.family: "Consolas"
            Layout.alignment: Qt.AlignHCenter
        }

        // 分隔线
        Rectangle {
            Layout.fillWidth: true; height: 1; color: "#21262d"
        }

        // 状态指示
        Rectangle {
            Layout.fillWidth: true; height: 32; radius: 6
            color: "#0d1117"

            RowLayout {
                anchors.centerIn: parent
                spacing: 6

                Rectangle {
                    width: 8; height: 8; radius: 4
                    color: stageCard.isExecuting ? "#d29922" : (stageCard.timerRunning ? "#3fb950" : "#484f58")
                }
                Text {
                    text: stageCard.isExecuting ? "执行中" : (stageCard.timerRunning ? "可执行" : "等待")
                    color: stageCard.isExecuting ? "#d29922" : (stageCard.timerRunning ? "#3fb950" : "#484f58")
                    font.pixelSize: 13; font.bold: true
                }
            }
        }

        // 反馈文本区
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: 8
            color: "#0d1117"
            clip: true

            Flickable {
                anchors.fill: parent
                anchors.margins: 8
                contentWidth: feedbackLabel.width
                contentHeight: feedbackLabel.height
                flickableDirection: Flickable.VerticalFlick

                Text {
                    id: feedbackLabel
                    width: parent.width
                    text: stageCard.feedbackText || "—"
                    color: stageCard.feedbackText ? "#c9d1d9" : "#484f58"
                    font.pixelSize: 12; font.family: "Consolas"
                    wrapMode: Text.WordWrap
                }
            }
        }

        // 按钮区
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 52
            Layout.minimumHeight: 52
            Layout.maximumHeight: 52
            spacing: 8

            // 梅林区双按钮：修改标定 + 执行任务
            Rectangle {
                visible: stageCard.stageId === 2
                Layout.fillWidth: true; Layout.fillHeight: true; radius: 12
                color: calibArea.pressed ? Qt.darker("#3fb950", 1.2) : "#3fb950"
                opacity: (stageCard.estopActive || stageCard.isExecuting) ? 0.6 : 1.0

                scale: calibArea.pressed ? 0.92 : 1.0
                Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                Behavior on color { ColorAnimation { duration: 80 } }

                Rectangle {
                    anchors.fill: parent; radius: 12
                    color: "#ffffff"
                    opacity: calibArea.pressed ? 0.12 : 0
                    Behavior on opacity { NumberAnimation { duration: 60 } }
                }

                TapHandler {
                    id: calibArea
                    enabled: !stageCard.estopActive
                    onTapped: stageCard.calibrateClicked(stageCard.stageId)
                }

                Text {
                    anchors.centerIn: parent
                    text: "🔧  修改标定"
                    color: "#fff"; font.pixelSize: 18; font.bold: true
                }
            }

            Rectangle {
                visible: stageCard.stageId === 2
                Layout.fillWidth: true; Layout.fillHeight: true; radius: 12
                color: {
                    if (stageCard.estopActive) return "#21262d"
                    if (stageCard.isExecuting) return Qt.darker("#3fb950", 1.4)
                    return execArea2.pressed ? Qt.darker("#3fb950", 1.2) : "#3fb950"
                }
                opacity: (stageCard.estopActive || stageCard.isExecuting) ? 0.6 : 1.0

                scale: execArea2.pressed ? 0.92 : 1.0
                Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                Behavior on color { ColorAnimation { duration: 80 } }

                Rectangle {
                    anchors.fill: parent; radius: 12
                    color: "#ffffff"
                    opacity: execArea2.pressed ? 0.12 : 0
                    Behavior on opacity { NumberAnimation { duration: 60 } }
                }

                TapHandler {
                    id: execArea2
                    enabled: !stageCard.isExecuting && !stageCard.estopActive
                    onTapped: {
                        stageCard.isExecuting = true
                        stageCard.executeClicked(stageCard.stageId)
                    }
                }

                Text {
                    anchors.centerIn: parent
                    text: stageCard.isExecuting ? "执行中..." : "▶  执行任务"
                    color: "#fff"; font.pixelSize: 18; font.bold: true
                }
            }

            // 非梅林区单按钮
            Rectangle {
                visible: stageCard.stageId !== 2
                Layout.fillWidth: true; Layout.fillHeight: true; radius: 12
                color: {
                    if (stageCard.estopActive) return "#21262d"
                    if (stageCard.isExecuting) return Qt.darker(stageCard.accentColor, 1.4)
                    return execArea.pressed ? Qt.lighter(stageCard.accentColor, 1.15) : stageCard.accentColor
                }
                opacity: (stageCard.estopActive || stageCard.isExecuting) ? 0.6 : 1.0

                scale: execArea.pressed ? 0.90 : 1.0
                Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                Behavior on color { ColorAnimation { duration: 80 } }

                Rectangle {
                    anchors.fill: parent; radius: 12
                    color: "#ffffff"
                    opacity: execArea.pressed ? 0.12 : 0
                    Behavior on opacity { NumberAnimation { duration: 60 } }
                }

                TapHandler {
                    id: execArea
                    enabled: !stageCard.isExecuting && !stageCard.estopActive
                    onTapped: {
                        if (stageCard.stageId === 3) {
                            stageCard.showCombatGrid = true
                            stageCard.isExecuting = true
                            stageCard.executeClicked(stageCard.stageId)
                            return
                        }
                        stageCard.isExecuting = true
                        stageCard.executeClicked(stageCard.stageId)
                    }
                }

                Text {
                    anchors.centerIn: parent
                    text: {
                        if (stageCard.estopActive) return "已急停"
                        if (stageCard.isExecuting && stageCard.stageId !== 3) return "执行中..."
                        if (stageCard.stageId === 3 && stageCard.showCombatGrid) return "🔧  重新选择目标"
                        return "▶  执行任务"
                    }
                    color: "#fff"; font.pixelSize: 18; font.bold: true
                }
            }
        }
    }

    // 对抗区九宫格
    CombatGrid {
        id: combatGrid
        visible: stageCard.showCombatGrid
        anchors.fill: parent
        anchors.margins: 8
        accentColor: stageCard.accentColor

        onExecuteRequested: {
            stageCard.isExecuting = true
            stageCard.showCombatGrid = false
            stageCard.executeClicked(stageCard.stageId)
        }
    }

    // 执行中边框脉动
    SequentialAnimation on border.width {
        running: stageCard.isExecuting && !stageCard.showCombatGrid
        loops: Animation.Infinite
        NumberAnimation { to: 4; duration: 500; easing.type: Easing.InOutQuad }
        NumberAnimation { to: 2; duration: 500; easing.type: Easing.InOutQuad }
    }

    // 收到结果后停止执行状态
    Connections {
        target: HmiController
        function onStageCompleted(completed) {
            if (completed) {
                stageCard.isExecuting = false
            }
        }
    }
}
