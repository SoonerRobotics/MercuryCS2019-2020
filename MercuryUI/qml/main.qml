import QtQuick 2.9
import QtQuick.Controls 2.2

ApplicationWindow {
    id: window

    visible: true
    title: qsTr("Pong")
    minimumHeight: 500
    minimumWidth: 510

    Button {
        height: 60

        text: "Win"
        font.pointSize: 22

    }

}