// DashboardPage.qml - 赛中看板
// 三张赛段卡片：武馆区 / 梅林区 / 对抗区

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "components"

Item {
    id: dashPage

    property string stageFeedbackText: ""
    property bool estopActive: false
    property bool timerRunning: false

    signal executeStage(int stageId)
    signal backToCalibration()

    Rectangle {
        anchors.fill: parent
        color: "#0d1117"

        RowLayout {
            anchors.fill: parent
            anchors.margins: 14
            anchors.bottomMargin: 80
            spacing: 14

            StageCard {
                Layout.fillWidth: true; Layout.fillHeight: true
                title: "武馆区"
                subtitle: "Martial Club"
                icon: "武"
                stageId: 1
                accentColor: "#f0883e"
                feedbackText: dashPage.stageFeedbackText
                estopActive: dashPage.estopActive
                timerRunning: dashPage.timerRunning
                onExecuteClicked: function(sid) { dashPage.executeStage(sid) }
            }

            StageCard {
                Layout.fillWidth: true; Layout.fillHeight: true
                title: "梅林区"
                subtitle: "Meihua Forest"
                icon: "梅"
                stageId: 2
                accentColor: "#3fb950"
                feedbackText: dashPage.stageFeedbackText
                estopActive: dashPage.estopActive
                timerRunning: dashPage.timerRunning
                onCalibrateClicked: function(sid) {
                    dashPage.backToCalibration()
                }
                onExecuteClicked: function(sid) {
                    HmiController.executeStage(sid)
                }
            }

            StageCard {
                Layout.fillWidth: true; Layout.fillHeight: true
                title: "对抗区"
                subtitle: "Arena"
                icon: "竞"
                stageId: 3
                accentColor: "#58a6ff"
                feedbackText: dashPage.stageFeedbackText
                estopActive: dashPage.estopActive
                timerRunning: dashPage.timerRunning
                onExecuteClicked: function(sid) { dashPage.executeStage(sid) }
            }
        }
    }
}