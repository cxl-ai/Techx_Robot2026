pragma Singleton
import QtQuick 2.15

QtObject {
    // ── 暗黑主题背景 ──
    readonly property color bgMain: "#0d1117"
    readonly property color bgCard: "#161b22"
    readonly property color bgDark: "#21262d"
    readonly property color bgInput: "#0d1117"

    // ── 边框 ──
    readonly property color borderDefault: "#30363d"

    // ── 强调色 ──
    readonly property color accentGreen: "#3fb950"
    readonly property color accentRed: "#f85149"
    readonly property color accentBlue: "#58a6ff"
    readonly property color accentOrange: "#f0883e"
    readonly property color accentAmber: "#d29922"

    // ── 文字 ──
    readonly property color textPrimary: "#c9d1d9"
    readonly property color textSecondary: "#8b949e"
    readonly property color textMuted: "#484f58"
    readonly property color textInverse: "#ffffff"
}
