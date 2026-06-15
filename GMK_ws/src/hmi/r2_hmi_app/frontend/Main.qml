// Main.qml - R2 HMI 主入口
// 暗黑工业风触控UI - ROBOCON 2026 "武林探秘"

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

ApplicationWindow {
    id: root
    visible: true
    width: 1024
    height: 600
    title: "R2 HMI - ROBOCON 2026"
    color: "#0d1117"

    // ── 全局状态 ────────────────────────────────────────────────
    property int currentPage: 0
    property int teamColor: -1
    property bool estopActive: false
    property int matchTimerSeconds: 180
    property bool timerRunning: false
    property string stageFeedbackText: ""
    property bool robotConnected: false
    property var meihuaStates: [0,0,0,0,0,0,0,0,0,0,0,0]
    property int brushMode: 1

    // 页面名称列表
    property var pageNames: ["阵营选择", "标定设置", "赛中看板"]

    function goBack() {
        if (root.currentPage > 0) {
            root.currentPage = root.currentPage - 1
            // 回到标定页时，倒计时应该还在跑（如果启动了的话），不重置
            if (root.currentPage === 0) {
                HmiController.resetMatchTimer()
            }
        }
    }

    // ── 顶部栏 ──────────────────────────────────────────────────
    header: Rectangle {
        height: 50
        color: "#161b22"
        border.color: "#30363d"
        border.width: 1

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 14
            anchors.rightMargin: 14
            spacing: 12

            // 返回按钮 - 更明显
            Rectangle {
                visible: root.currentPage > 0
                width: 40; height: 36; radius: 10
                color: backArea.pressed ? "#30363d" : "#21262d"
                border.color: "#58a6ff"
                border.width: 1.5

                scale: backArea.pressed ? 0.93 : 1.0
                Behavior on scale { NumberAnimation { duration: 80 } }

                TapHandler {
                    id: backArea
                    onTapped: root.goBack()
                }

                Text {
                    anchors.centerIn: parent
                    text: "←"
                    color: "#58a6ff"
                    font.pixelSize: 22
                    font.bold: true
                }
            }

            // 连接状态
            Rectangle {
                width: 10; height: 10; radius: 5
                color: root.robotConnected ? "#3fb950" : "#f85149"
                Layout.alignment: Qt.AlignVCenter
            }
            Text {
                text: root.robotConnected ? "ONLINE" : "OFFLINE"
                color: root.robotConnected ? "#3fb950" : "#f85149"
                font.pixelSize: 12; font.bold: true; font.family: "Consolas"
            }

            Item { Layout.fillWidth: true }

            // 阵营标识
            Rectangle {
                visible: root.teamColor >= 0
                width: 56; height: 26; radius: 6
                color: root.teamColor === 0 ? "#da3633" : "#1f6feb"
                Text {
                    anchors.centerIn: parent
                    text: root.teamColor === 0 ? "红方" : "蓝方"
                    color: "#fff"; font.pixelSize: 12; font.bold: true
                }
            }

            // 页面指示器
            RowLayout {
                spacing: 6
                Repeater {
                    model: 3
                    Rectangle {
                        width: 10; height: 10; radius: 5
                        color: root.currentPage === index ? "#58a6ff" : "#30363d"
                    }
                }
            }
            Text {
                text: root.pageNames[root.currentPage] || ""
                color: "#8b949e"
                font.pixelSize: 12
            }

            // 倒计时 - 仅在赛中看板页显示
            Rectangle {
                visible: root.currentPage === 2
                width: 120; height: 36; radius: 10
                color: root.timerRunning ? (root.matchTimerSeconds <= 30 ? "#3d1111" : "#0d2137") : "#21262d"
                border.color: root.timerRunning ? (root.matchTimerSeconds <= 30 ? "#f85149" : "#58a6ff") : "#30363d"
                border.width: root.timerRunning ? 2.5 : 1

                Text {
                    anchors.centerIn: parent
                    text: {
                        var m = Math.floor(root.matchTimerSeconds / 60)
                        var s = root.matchTimerSeconds % 60
                        return (m < 10 ? "0" : "") + m + ":" + (s < 10 ? "0" : "") + s
                    }
                    color: root.timerRunning ? "#fff" : "#484f58"
                    font.pixelSize: 22; font.bold: true; font.family: "Consolas"
                }
            }
        }
    }

    // ── 页面堆栈 ────────────────────────────────────────────────
    StackLayout {
        id: pageStack
        anchors.fill: parent
        currentIndex: root.currentPage

        BootPage {
            onTeamSelected: function(color) {
                root.teamColor = color
                root.currentPage = 1
            }
        }

        CalibrationPage {
            id: calibPage
            brushMode: root.brushMode
            teamColor: root.teamColor

            // 首次进入时从 root 加载已有的标定数据
            Component.onCompleted: {
                calibPage.meihuaStates = root.meihuaStates.slice()
            }

            onBrushModeChanged: { root.brushMode = calibPage.brushMode }
            onMeihuaStateChanged: function(index, newState) {
                root.meihuaStates[index] = newState
            }
            onMeihuaClicked: function(index) {
                HmiController.setMeihuaState(index, calibPage.meihuaStates[index])
            }
            onCalibrationConfirmed: { root.currentPage = 2 }
        }

        DashboardPage {
            stageFeedbackText: root.stageFeedbackText
            estopActive: root.estopActive
            timerRunning: root.timerRunning

            onExecuteStage: function(stageId) {
                HmiController.executeStage(stageId)
            }
            onBackToCalibration: {
                root.currentPage = 1
            }
        }
    }

    // ── 全局急停按钮 ────────────────────────────────────────────
    Rectangle {
        id: estopBtn
        z: 100
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: 18
        anchors.bottomMargin: 18
        width: 88; height: 88; radius: 44
        color: root.estopActive ? "#ff0000" : "#8b0000"
        border.color: "#fff"; border.width: 3

        SequentialAnimation on scale {
            running: root.estopActive; loops: Animation.Infinite
            NumberAnimation { to: 1.13; duration: 350; easing.type: Easing.InOutQuad }
            NumberAnimation { to: 1.0; duration: 350; easing.type: Easing.InOutQuad }
        }

        Text {
            anchors.centerIn: parent
            text: "急停"
            color: "#fff"; font.pixelSize: 18; font.bold: true
        }

        TapHandler {
            onTapped: {
                root.estopActive = !root.estopActive
                HmiController.requestEstop(root.estopActive)
            }
        }
    }

    // ── HmiController 信号 ──────────────────────────────────────
    Connections {
        target: HmiController

        function onEstopChanged(active) { root.estopActive = active }
        function onMatchTimerUpdated(seconds) { root.matchTimerSeconds = seconds }
        function onTimerStarted(running) { root.timerRunning = running }
        function onStageFeedbackUpdated(text) { root.stageFeedbackText = text }
        function onConnectionChanged(connected) { root.robotConnected = connected }

        function onCalibrationResponse(success, message) {
            toast.show(success ? "标定下发成功" : "标定失败: " + message, success ? "#3fb950" : "#f85149")
        }
    }

    // ── Toast 通知 ──────────────────────────────────────────────
    Rectangle {
        id: toast
        z: 200
        anchors.horizontalCenter: parent.horizontalCenter
        y: 60
        width: toastText.contentWidth + 40
        height: 40
        radius: 10
        color: "#21262d"
        border.color: toast._color
        border.width: 1
        opacity: 0
        visible: opacity > 0

        property color _color: "#3fb950"

        function show(msg, color) {
            toastText.text = msg; toast._color = color
            toast.opacity = 1; toastTimer.restart()
        }

        Text {
            id: toastText
            anchors.centerIn: parent
            color: toast._color
            font.pixelSize: 14; font.bold: true
        }

        Timer {
            id: toastTimer; interval: 2500
            onTriggered: toast.opacity = 0
        }

        Behavior on opacity { NumberAnimation { duration: 300 } }
    }
}