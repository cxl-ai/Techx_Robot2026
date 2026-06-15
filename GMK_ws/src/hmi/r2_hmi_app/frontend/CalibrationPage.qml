// CalibrationPage.qml - 梅林区标定页
// 俯视图风格 - 4行×3列网格，红/蓝区镜像对称

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: calibPage

    property var meihuaStates: [0,0,0,0,0,0,0,0,0,0,0,0]
    property int brushMode: 1
    property int teamColor: -1

    onMeihuaStatesChanged: calibPage.recomputePath()

    signal meihuaClicked(int index)
    signal calibrationConfirmed()
    signal meihuaStateChanged(int index, int newState)

    // ── A* 最短路径 ──
    property string pathCells: ""

    function recomputePath() {
        var states = calibPage.meihuaStates
        var rows = 4, cols = 3

        // 找到所有可用的入口和出口
        var entries = []
        var exits = []
        for (var c = 0; c < cols; c++) {
            if (states[c] !== 2) entries.push(c)
            if (states[9 + c] !== 2) exits.push(9 + c)
        }
        if (entries.length === 0 || exits.length === 0) {
            calibPage.pathCells = ""
            return
        }

        // 高度代价映射
        function moveCost(idx) {
            var h = calibPage.effectiveHeight(idx)
            if (h === 0) return 1    // 200H
            if (h === 1) return 2    // 400H
            return 3                 // 600H
        }

        // 启发式：到出口行的曼哈顿距离
        function heuristic(idx) {
            var r = Math.floor(idx / cols)
            return Math.abs(r - 3) * 2
        }

        // 获取邻居
        function neighbors(idx) {
            var r = Math.floor(idx / cols)
            var c = idx % cols
            var nb = []
            if (r > 0) nb.push(idx - cols)   // 上
            if (r < rows - 1) nb.push(idx + cols)  // 下
            if (c > 0) nb.push(idx - 1)      // 左
            if (c < cols - 1) nb.push(idx + 1)  // 右
            return nb
        }

        // A* 从多个入口到最近出口
        var bestPath = null
        var bestCost = Infinity

        for (var ei = 0; ei < entries.length; ei++) {
            var start = entries[ei]
            var openSet = [start]
            var cameFrom = {}
            var gScore = {}
            var fScore = {}
            gScore[start] = 0
            fScore[start] = heuristic(start)

            while (openSet.length > 0) {
                // 找 fScore 最小的
                var minF = Infinity
                var minIdx = 0
                for (var oi = 0; oi < openSet.length; oi++) {
                    var f = fScore[openSet[oi]] || Infinity
                    if (f < minF) { minF = f; minIdx = oi }
                }
                var current = openSet[minIdx]
                openSet.splice(minIdx, 1)

                // 到达出口？
                if (current >= 9 && current <= 11 && states[current] !== 2) {
                    // 重建路径
                    var path = [current]
                    var p = current
                    while (cameFrom.hasOwnProperty(String(p))) {
                        p = cameFrom[String(p)]
                        path.unshift(p)
                    }
                    var totalCost = gScore[current] || Infinity
                    if (totalCost < bestCost) {
                        bestCost = totalCost
                        bestPath = path
                    }
                    break
                }

                var nbs = neighbors(current)
                for (var ni = 0; ni < nbs.length; ni++) {
                    var nb = nbs[ni]
                    if (states[nb] === 2) continue  // 假 KFS 不可通行
                    var tentG = (gScore[current] || Infinity) + moveCost(nb)
                    if (tentG < (gScore[nb] || Infinity)) {
                        cameFrom[String(nb)] = current
                        gScore[nb] = tentG
                        fScore[nb] = tentG + heuristic(nb)
                        if (openSet.indexOf(nb) < 0) openSet.push(nb)
                    }
                }
            }
        }

        calibPage.pathCells = bestPath !== null ? bestPath.join(",") : ""
    }

    // ═══════════════════════════════════════════════════════════════
    // 场地数据：4行×3列 = 12个方块，每格1200mm×1200mm
    // 红区高度分布：
    //   Row 0 (入口): 1:400H  2:200H  3:400H
    //   Row 1:        4:200H  5:400H  6:600H
    //   Row 2:        7:400H  8:600H  9:400H
    //   Row 3 (出口): 10:200H 11:400H 12:200H
    //
    // 蓝区 = 红区左右镜像（第1列与第3列高度互换，第2列不变）
    //   蓝区: 4号→600H, 6号→200H
    // ═══════════════════════════════════════════════════════════════

    // 红区高度数组 (0=200H, 1=400H, 2=600H)
    property var redHeights: [1,0,1, 0,1,2, 1,2,1, 0,1,0]

    // 颜色映射：200H=深绿, 400H=中绿, 600H=黄绿
    property color bgBase: "#b4d2b4"      // 基底浅灰绿
    property color lineDark: "#1a661a"    // 深绿色边线
    property color c200H: "#2e7d32"       // 200H 深绿
    property color c400H: "#4caf50"       // 400H 中绿
    property color c600H: "#c0ca33"       // 600H 黄绿

    // 获取当前阵营下的有效高度级别
    function effectiveHeight(idx) {
        var h = redHeights[idx]
        // 蓝区镜像：第1列(0,3,6,9) 与 第3列(2,5,8,11) 互换
        if (calibPage.teamColor === 1) {
            var col = idx % 3
            if (col === 0) h = redHeights[idx + 2]  // 取第3列
            else if (col === 2) h = redHeights[idx - 2]  // 取第1列
        }
        return h
    }

    function getHeightColor(idx) {
        var h = effectiveHeight(idx)
        if (h === 0) return c200H
        if (h === 1) return c400H
        return c600H
    }

    function getHeightLabel(idx) {
        var h = effectiveHeight(idx)
        if (h === 0) return "200"
        if (h === 1) return "400"
        return "600"
    }

    Rectangle {
        anchors.fill: parent
        color: "#0d1117"

        RowLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 14

            // ── 左侧工具面板 ──────────────────────────────────
            Rectangle {
                Layout.preferredWidth: 170
                Layout.fillHeight: true
                radius: 14
                color: "#161b22"
                border.color: "#30363d"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 10

                    Text {
                        text: "画笔工具"
                        color: "#58a6ff"
                        font.pixelSize: 17; font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }

                    Rectangle {
                        Layout.fillWidth: true; height: 1; color: "#30363d"
                    }

                    // 真 KFS 画笔
                    Rectangle {
                        Layout.fillWidth: true; Layout.preferredHeight: 80; radius: 12
                        color: realBrushArea.pressed ? "#1a4a2a" : (calibPage.brushMode === 1 ? "#0d3320" : "#161b22")
                        border.color: calibPage.brushMode === 1 ? "#3fb950" : "#30363d"
                        border.width: calibPage.brushMode === 1 ? 3 : 1

                        scale: realBrushArea.pressed ? 0.90 : 1.0
                        Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                        Behavior on color { ColorAnimation { duration: 80 } }

                        // 按压闪光层
                        Rectangle {
                            anchors.fill: parent; radius: 12
                            color: "#ffffff"
                            opacity: realBrushArea.pressed ? 0.08 : 0
                            Behavior on opacity { NumberAnimation { duration: 60 } }
                        }

                        TapHandler {
                            id: realBrushArea
                            onTapped: calibPage.brushMode = calibPage.brushMode === 1 ? 0 : 1
                        }

                        ColumnLayout {
                            anchors.centerIn: parent; spacing: 4
                            Rectangle {
                                width: 36; height: 36; radius: 18
                                color: "#3fb950"; border.color: "#7ee787"; border.width: 2
                                Layout.alignment: Qt.AlignHCenter
                                Text {
                                    anchors.centerIn: parent
                                    text: "✓"; color: "#fff"; font.pixelSize: 18; font.bold: true
                                }
                            }
                            Text {
                                text: "真 R2 KFS"
                                color: calibPage.brushMode === 1 ? "#3fb950" : "#8b949e"
                                font.pixelSize: 13; font.bold: calibPage.brushMode === 1
                                Layout.alignment: Qt.AlignHCenter
                            }
                        }
                    }

                    // 假 KFS 画笔
                    Rectangle {
                        Layout.fillWidth: true; Layout.preferredHeight: 80; radius: 12
                        color: fakeBrushArea.pressed ? "#4a1a1a" : (calibPage.brushMode === 2 ? "#330a0a" : "#161b22")
                        border.color: calibPage.brushMode === 2 ? "#f85149" : "#30363d"
                        border.width: calibPage.brushMode === 2 ? 3 : 1

                        scale: fakeBrushArea.pressed ? 0.90 : 1.0
                        Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                        Behavior on color { ColorAnimation { duration: 80 } }

                        // 按压闪光层
                        Rectangle {
                            anchors.fill: parent; radius: 12
                            color: "#ffffff"
                            opacity: fakeBrushArea.pressed ? 0.08 : 0
                            Behavior on opacity { NumberAnimation { duration: 60 } }
                        }

                        TapHandler {
                            id: fakeBrushArea
                            onTapped: calibPage.brushMode = calibPage.brushMode === 2 ? 0 : 2
                        }

                        ColumnLayout {
                            anchors.centerIn: parent; spacing: 4
                            Rectangle {
                                width: 36; height: 36; radius: 18
                                color: "#f85149"; border.color: "#ff7b72"; border.width: 2
                                Layout.alignment: Qt.AlignHCenter
                                Text {
                                    anchors.centerIn: parent
                                    text: "✗"; color: "#fff"; font.pixelSize: 18; font.bold: true
                                }
                            }
                            Text {
                                text: "假 R2 KFS"
                                color: calibPage.brushMode === 2 ? "#f85149" : "#8b949e"
                                font.pixelSize: 13; font.bold: calibPage.brushMode === 2
                                Layout.alignment: Qt.AlignHCenter
                            }
                        }
                    }

                    // R1 KFS 画笔（安全绕行）
                    Rectangle {
                        Layout.fillWidth: true; Layout.preferredHeight: 80; radius: 12
                        color: r1BrushArea.pressed ? "#0d2137" : (calibPage.brushMode === 3 ? "#0d1a3d" : "#161b22")
                        border.color: calibPage.brushMode === 3 ? "#1f6feb" : "#30363d"
                        border.width: calibPage.brushMode === 3 ? 3 : 1

                        scale: r1BrushArea.pressed ? 0.90 : 1.0
                        Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                        Behavior on color { ColorAnimation { duration: 80 } }

                        // 按压闪光层
                        Rectangle {
                            anchors.fill: parent; radius: 12
                            color: "#ffffff"
                            opacity: r1BrushArea.pressed ? 0.08 : 0
                            Behavior on opacity { NumberAnimation { duration: 60 } }
                        }

                        TapHandler {
                            id: r1BrushArea
                            onTapped: calibPage.brushMode = calibPage.brushMode === 3 ? 0 : 3
                        }

                        ColumnLayout {
                            anchors.centerIn: parent; spacing: 4
                            Rectangle {
                                width: 36; height: 36; radius: 18
                                color: "#1f6feb"; border.color: "#58a6ff"; border.width: 2
                                Layout.alignment: Qt.AlignHCenter
                                Text {
                                    anchors.centerIn: parent
                                    text: "R1"; color: "#fff"; font.pixelSize: 12; font.bold: true
                                }
                            }
                            Text {
                                text: "R1 KFS"
                                color: calibPage.brushMode === 3 ? "#58a6ff" : "#8b949e"
                                font.pixelSize: 13; font.bold: calibPage.brushMode === 3
                                Layout.alignment: Qt.AlignHCenter
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }

                    // 清除全部按钮
                    Rectangle {
                        Layout.fillWidth: true; Layout.preferredHeight: 46; radius: 12
                        color: clearAllArea.pressed ? "#3a2020" : "#1a0a0a"
                        border.color: clearAllArea.pressed ? "#ff6b6b" : "#f85149"
                        border.width: 2

                        scale: clearAllArea.pressed ? 0.90 : 1.0
                        Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
                        Behavior on color { ColorAnimation { duration: 80 } }

                        Rectangle {
                            anchors.fill: parent; radius: 12
                            color: "#ffffff"
                            opacity: clearAllArea.pressed ? 0.1 : 0
                            Behavior on opacity { NumberAnimation { duration: 60 } }
                        }

                        TapHandler {
                            id: clearAllArea
                            onTapped: {
                                calibPage.meihuaStates = [0,0,0,0,0,0,0,0,0,0,0,0]
                                calibPage.recomputePath()
                                for (var i = 0; i < 12; i++) {
                                    calibPage.meihuaStateChanged(i, 0)
                                    calibPage.meihuaClicked(i)
                                }
                            }
                        }

                        Text {
                            anchors.centerIn: parent
                            text: "✕  清除全部标记"
                            color: "#f85149"; font.pixelSize: 15; font.bold: true
                        }
                    }

                    // 统计面板
                    Rectangle {
                        id: statsBox
                        Layout.fillWidth: true; height: 82; radius: 10
                        color: "#0d1117"; border.color: "#30363d"; border.width: 1

                        function countState(s) {
                            var c = 0
                            for (var i = 0; i < 12; i++) {
                                if (calibPage.meihuaStates[i] === s) c++
                            }
                            return c
                        }

                        ColumnLayout {
                            anchors.centerIn: parent; spacing: 4
                            RowLayout {
                                spacing: 10
                                ColumnLayout {
                                    spacing: 0
                                    Text { text: "真"; color: "#3fb950"; font.pixelSize: 10; font.bold: true
                                        Layout.alignment: Qt.AlignHCenter }
                                    Text { text: statsBox.countState(1); color: "#3fb950"
                                        font.pixelSize: 18; font.bold: true; font.family: "Consolas"
                                        Layout.alignment: Qt.AlignHCenter }
                                }
                                Text { text: "/"; color: "#484f58"; font.pixelSize: 14 }
                                ColumnLayout {
                                    spacing: 0
                                    Text { text: "假"; color: "#f85149"; font.pixelSize: 10; font.bold: true
                                        Layout.alignment: Qt.AlignHCenter }
                                    Text { text: statsBox.countState(2); color: "#f85149"
                                        font.pixelSize: 18; font.bold: true; font.family: "Consolas"
                                        Layout.alignment: Qt.AlignHCenter }
                                }
                            }
                            RowLayout {
                                spacing: 10
                                ColumnLayout {
                                    spacing: 0
                                    Text { text: "R1"; color: "#58a6ff"; font.pixelSize: 10; font.bold: true
                                        Layout.alignment: Qt.AlignHCenter }
                                    Text { text: statsBox.countState(3); color: "#58a6ff"
                                        font.pixelSize: 18; font.bold: true; font.family: "Consolas"
                                        Layout.alignment: Qt.AlignHCenter }
                                }
                                Text { text: "/"; color: "#484f58"; font.pixelSize: 14 }
                                ColumnLayout {
                                    spacing: 0
                                    Text { text: "空"; color: "#8b949e"; font.pixelSize: 10; font.bold: true
                                        Layout.alignment: Qt.AlignHCenter }
                                    Text { text: statsBox.countState(0); color: "#8b949e"
                                        font.pixelSize: 18; font.bold: true; font.family: "Consolas"
                                        Layout.alignment: Qt.AlignHCenter }
                                }
                            }
                        }
                    }
                }
            }

            // ── 中央：梅林网格（俯视图，4行×3列）───────────────────
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: 14
                color: calibPage.bgBase
                border.color: calibPage.lineDark
                border.width: 2

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8

                    // 标题行
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        Text {
                            text: "梅林区 · R2 标定 · 每格 1200×1200mm"
                            color: calibPage.lineDark
                            font.pixelSize: 16; font.bold: true
                        }
                        Item { Layout.fillWidth: true }
                        RowLayout { spacing: 6
                            Repeater {
                                model: [
                                    { h: "200H", c: c200H },
                                    { h: "400H", c: c400H },
                                    { h: "600H", c: c600H }
                                ]
                                RowLayout { spacing: 4
                                    Rectangle {
                                        width: 14; height: 14; radius: 2
                                        color: modelData.c
                                        border.color: lineDark; border.width: 1
                                    }
                                    Text {
                                        text: modelData.h; color: lineDark; font.pixelSize: 11
                                    }
                                }
                            }
                        }
                    }

                    // 网格容器
                    Item {
                        Layout.fillWidth: true
                        Layout.fillHeight: true

                        // R2 入口区
                        Text {
                            text: "R2 入口区"
                            color: calibPage.lineDark
                            font.pixelSize: 15; font.bold: true
                            anchors.top: parent.top; anchors.topMargin: 2
                            anchors.horizontalCenter: parent.horizontalCenter
                        }

                        // 中心 "树林" 水印
                        Text {
                            text: "树林"
                            color: "#ffffff"; font.pixelSize: 36; font.bold: true
                            anchors.centerIn: parent; opacity: 0.35
                        }

                        // R2 出口区
                        Text {
                            text: "R2 出口区"
                            color: calibPage.lineDark
                            font.pixelSize: 15; font.bold: true
                            anchors.bottom: parent.bottom; anchors.bottomMargin: 2
                            anchors.horizontalCenter: parent.horizontalCenter
                        }

                        // 4行×3列网格
                        GridLayout {
                            anchors.centerIn: parent
                            columns: 3
                            columnSpacing: 8
                            rowSpacing: 8

                            Repeater {
                                model: 12

                                // 每个方块：外部白色方框 + 内部高度色块 + 梅花桩标记
                                Item {
                                    width: 90; height: 90

                                    property int cellState: calibPage.meihuaStates[index]
                                    property int colIdx: index % 3
                                    property bool isPathCell: ("," + calibPage.pathCells + ",").indexOf("," + index + ",") >= 0

                                    // 整体缩放（触控猛击反馈）
                                    scale: cellTapArea.pressed ? 0.88 : (isPathCell ? 1.06 : 1.0)
                                    Behavior on scale { NumberAnimation { duration: 80; easing.type: Easing.OutBack } }

                                    // 路径发光环（最外层）
                                    Rectangle {
                                        visible: isPathCell
                                        anchors.fill: parent
                                        anchors.margins: -3
                                        color: "transparent"
                                        border.color: "#ffd700"
                                        border.width: 3
                                        radius: 4

                                        SequentialAnimation on opacity {
                                            running: isPathCell; loops: Animation.Infinite
                                            NumberAnimation { to: 0.5; duration: 350; easing.type: Easing.InOutQuad }
                                            NumberAnimation { to: 1.0; duration: 350; easing.type: Easing.InOutQuad }
                                        }
                                    }

                                    // 外部白色方框（指示 KFS 放置区）
                                    Rectangle {
                                        anchors.fill: parent
                                        color: cellTapArea.pressed ? "#3d3d3d" : "transparent"
                                        border.color: {
                                            if (isPathCell) return "#ffd700"
                                            if (cellState === 1) return "#3fb950"
                                            if (cellState === 2) return "#f85149"
                                            if (cellState === 3) return "#58a6ff"
                                            return "#ffffff"
                                        }
                                        border.width: (isPathCell || cellState > 0) ? 3.5 : 2.5
                                        radius: 2

                                        Behavior on border.width { NumberAnimation { duration: 120 } }

                                        // 高度色块（内缩，让白框可见）
                                        Rectangle {
                                            anchors.fill: parent
                                            anchors.margins: 3
                                            color: calibPage.getHeightColor(index)
                                            radius: 1

                                            // 中心细线方框（梅花桩物理台面标记）
                                            Rectangle {
                                                anchors.centerIn: parent
                                                width: parent.width * 0.5
                                                height: parent.height * 0.5
                                                color: "transparent"
                                                border.color: "#ffffff"
                                                border.width: 1
                                                opacity: 0.6
                                            }

                                            // KFS 状态覆盖层
                                            Rectangle {
                                                visible: cellState > 0
                                                anchors.fill: parent
                                                color: {
                                                    if (cellState === 1) return "#3fb950"
                                                    if (cellState === 2) return "#f85149"
                                                    if (cellState === 3) return "#1f6feb"
                                                    return "transparent"
                                                }
                                                opacity: 0.30
                                            }

                                            // 状态文字
                                            Text {
                                                visible: cellState > 0
                                                anchors.centerIn: parent
                                                text: {
                                                    if (cellState === 1) return "真"
                                                    if (cellState === 2) return "假"
                                                    if (cellState === 3) return "R1"
                                                    return ""
                                                }
                                                color: "#ffffff"
                                                font.pixelSize: 15; font.bold: true
                                            }

                                            // 点击闪光
                                            Rectangle {
                                                id: clickFlash
                                                anchors.fill: parent
                                                color: "#ffffff"; opacity: 0
                                            }
                                        }
                                    }

                                    // 编号标签
                                    Text {
                                        text: (index + 1).toString()
                                        color: cellState > 0 ? "#ffffff" : calibPage.lineDark
                                        font.pixelSize: 10; font.bold: true
                                        anchors.top: parent.top; anchors.left: parent.left
                                        anchors.topMargin: 1; anchors.leftMargin: 3
                                    }

                                    // 高度标签
                                    Text {
                                        text: calibPage.getHeightLabel(index) + "H"
                                        color: cellState > 0 ? "#ffffff" : calibPage.lineDark
                                        font.pixelSize: 9
                                        anchors.bottom: parent.bottom; anchors.right: parent.right
                                        anchors.bottomMargin: 1; anchors.rightMargin: 3
                                    }

                                    // 点击闪光动画
                                    SequentialAnimation {
                                        id: flashAnim
                                        PropertyAction { target: clickFlash; property: "opacity"; value: 0.7 }
                                        NumberAnimation {
                                            target: clickFlash; property: "opacity"
                                            from: 0.7; to: 0; duration: 450
                                            easing.type: Easing.OutCubic
                                        }
                                    }

                                    TapHandler {
                                        id: cellTapArea
                                        onTapped: {
                                            flashAnim.start()
                                            var newStates = calibPage.meihuaStates.slice()
                                            if (calibPage.brushMode === 1) {
                                                newStates[index] = newStates[index] === 1 ? 0 : 1
                                            } else if (calibPage.brushMode === 2) {
                                                newStates[index] = newStates[index] === 2 ? 0 : 2
                                            } else if (calibPage.brushMode === 3) {
                                                newStates[index] = newStates[index] === 3 ? 0 : 3
                                            } else {
                                                newStates[index] = 0
                                            }
                                            calibPage.meihuaStates = newStates
                                            calibPage.recomputePath()
                                            calibPage.meihuaStateChanged(index, newStates[index])
                                            calibPage.meihuaClicked(index)
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ── 底部：下发按钮 ───────────────────────────────────────────
    Rectangle {
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottomMargin: 12
        width: 320; height: 54; radius: 14
        color: confirmArea.pressed ? "#1a7f37" : "#21262d"
        border.color: confirmArea.pressed ? "#7ee787" : "#3fb950"
        border.width: confirmArea.pressed ? 3.5 : 2.5

        scale: confirmArea.pressed ? 0.92 : 1.0
        Behavior on scale { NumberAnimation { duration: 100; easing.type: Easing.OutBack } }
        Behavior on border.color { ColorAnimation { duration: 80 } }

        // 按压闪光层
        Rectangle {
            anchors.fill: parent; radius: 14
            color: "#ffffff"
            opacity: confirmArea.pressed ? 0.1 : 0
            Behavior on opacity { NumberAnimation { duration: 60 } }
        }

        TapHandler {
            id: confirmArea
            onTapped: {
                HmiController.sendCalibration()
                calibPage.calibrationConfirmed()
            }
        }

        RowLayout {
            anchors.centerIn: parent; spacing: 8
            Text { text: "✓"; color: "#3fb950"; font.pixelSize: 20; font.bold: true }
            Text { text: "下发并锁定标定"; color: "#3fb950"; font.pixelSize: 20; font.bold: true }
        }
    }
}