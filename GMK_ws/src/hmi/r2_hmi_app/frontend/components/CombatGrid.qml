// CombatGrid.qml - 对抗区 3×3 九宫格战术面板
// 小屏适配，紧凑按钮

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: combatGrid

    property var selectedCells: []
    property color accentColor: "#58a6ff"

    signal cellsChanged(var indices)
    signal executeRequested()

    function isSelected(idx) {
        return combatGrid.selectedCells.indexOf(idx) >= 0
    }
    function selectedCount() {
        return combatGrid.selectedCells.length
    }
    function toggleCell(idx) {
        var arr = combatGrid.selectedCells.slice()
        var pos = arr.indexOf(idx)
        if (pos >= 0) {
            arr.splice(pos, 1)
        } else {
            arr.push(idx)
        }
        combatGrid.selectedCells = arr
        combatGrid.cellsChanged(arr)
    }
    function reset() {
        combatGrid.selectedCells = []
        combatGrid.cellsChanged([])
    }

    Rectangle {
        anchors.fill: parent
        color: "#0d1117"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 3
            spacing: 3

            // 3×3 网格
            GridLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumHeight: 120
                columns: 3
                columnSpacing: 4
                rowSpacing: 4

                Repeater {
                    model: 9

                    Rectangle {
                        id: cell
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        radius: 8

                        property bool isSel: combatGrid.isSelected(index)

                        color: {
                            if (cellArea.pressed) return "#2a3a4a"
                            if (isSel) return Qt.darker(combatGrid.accentColor, 1.6)
                            return "#161b22"
                        }

                        border.color: {
                            if (isSel) return combatGrid.accentColor
                            if (cellArea.pressed) return "#8b949e"
                            return "#30363d"
                        }
                        border.width: isSel ? 3 : 1.5

                        scale: cellArea.pressed ? 0.90 : (isSel ? 1.04 : 1.0)

                        Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                        Behavior on color { ColorAnimation { duration: 100 } }
                        Behavior on border.color { ColorAnimation { duration: 100 } }
                        Behavior on border.width { NumberAnimation { duration: 120 } }

                        Rectangle {
                            anchors.fill: parent; radius: 8
                            color: "#ffffff"
                            opacity: cellArea.pressed ? 0.1 : 0
                            Behavior on opacity { NumberAnimation { duration: 60 } }
                        }

                        // 选中闪光环
                        Rectangle {
                            visible: isSel
                            anchors.fill: parent; radius: 8
                            color: "transparent"
                            border.color: combatGrid.accentColor
                            border.width: 2
                            opacity: 0.6

                            SequentialAnimation on opacity {
                                running: isSel; loops: Animation.Infinite
                                NumberAnimation { to: 1.0; duration: 600; easing.type: Easing.InOutQuad }
                                NumberAnimation { to: 0.5; duration: 600; easing.type: Easing.InOutQuad }
                            }
                        }

                        ColumnLayout {
                            anchors.centerIn: parent
                            spacing: 2

                            Text {
                                text: (index + 1).toString()
                                color: isSel ? combatGrid.accentColor : "#8b949e"
                                font.pixelSize: 18; font.bold: true; font.family: "Consolas"
                                Layout.alignment: Qt.AlignHCenter
                                Behavior on color { ColorAnimation { duration: 150 } }
                            }

                            Rectangle {
                                visible: isSel
                                width: 22; height: 22; radius: 11
                                color: combatGrid.accentColor
                                Layout.alignment: Qt.AlignHCenter
                                Text {
                                    anchors.centerIn: parent
                                    text: "✓"
                                    color: "#fff"; font.pixelSize: 13; font.bold: true
                                }
                            }

                            Rectangle {
                                visible: !isSel
                                width: 22; height: 22; radius: 11
                                color: "#21262d"
                                Layout.alignment: Qt.AlignHCenter
                                Text {
                                    anchors.centerIn: parent
                                    text: "○"
                                    color: "#484f58"; font.pixelSize: 12
                                }
                            }
                        }

                        TapHandler {
                            id: cellArea
                            onTapped: combatGrid.toggleCell(index)
                        }
                    }
                }
            }

            // 执行任务 + 清除全部 按钮行
            RowLayout {
                Layout.fillWidth: true
                Layout.preferredHeight: 22
                spacing: 4

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    radius: 4
                    color: {
                        if (combatGrid.selectedCount() === 0) return "#21262d"
                        return execTap.pressed ? Qt.lighter(combatGrid.accentColor, 1.15) : combatGrid.accentColor
                    }
                    border.color: combatGrid.selectedCount() > 0 ? combatGrid.accentColor : "#30363d"
                    border.width: 1
                    opacity: combatGrid.selectedCount() === 0 ? 0.6 : 1.0

                    scale: execTap.pressed && combatGrid.selectedCount() > 0 ? 0.93 : 1.0
                    Behavior on scale { NumberAnimation { duration: 80; easing.type: Easing.OutBack } }

                    TapHandler {
                        id: execTap
                        enabled: combatGrid.selectedCount() > 0
                        onTapped: combatGrid.executeRequested()
                    }

                    Text {
                        anchors.centerIn: parent
                        text: "▶ 执行任务"
                        color: combatGrid.selectedCount() > 0 ? "#fff" : "#484f58"
                        font.pixelSize: 10
                        font.bold: true
                    }
                }

                Rectangle {
                    Layout.preferredWidth: 48
                    Layout.fillHeight: true
                    radius: 4
                    color: clearAllTap.pressed ? "#3a2020" : "#1a0a0a"
                    border.color: clearAllTap.pressed ? "#ff6b6b" : "#f85149"
                    border.width: 1
                    opacity: combatGrid.selectedCount() === 0 ? 0.6 : 1.0

                    scale: clearAllTap.pressed ? 0.92 : 1.0
                    Behavior on scale { NumberAnimation { duration: 80; easing.type: Easing.OutBack } }

                    TapHandler {
                        id: clearAllTap
                        enabled: combatGrid.selectedCount() > 0
                        onTapped: combatGrid.reset()
                    }

                    Text {
                        anchors.centerIn: parent
                        text: "✕ 清除"
                        color: combatGrid.selectedCount() > 0 ? "#f85149" : "#5a3030"
                        font.pixelSize: 10
                        font.bold: true
                    }
                }
            }
        }
    }
}