import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQuick.Window 2.2
import VideoPlayer 1.0

Window {    
    visible: true
    width: 640
    height: 480
    title: qsTr("Hello World")


    Rectangle{
        id: rectangle
        anchors.fill: parent
        color: "blue"

        RowLayout {
            width: parent.width * 0.9
            anchors.top: parent.top
            anchors.topMargin: 25
            anchors.horizontalCenter: parent.horizontalCenter

            Button {
                text: "Start"
                anchors.left: parent.left
                enabled: scanner.isConnected && !scanner.isScanning;
                onClicked: scanner.start()
            }

            Button {
                text: "Record"
                anchors.centerIn: parent
                enabled: false
            }

            Button {
                text: "Stop"
                enabled: scanner.isConnected && scanner.isScanning;
                onClicked: scanner.stop()
                anchors.right: parent.right
            }
        }
//        Rectangle{
//            width: 100
//            height: 100
//            color: scanner.isConnected ? "green" : "red"
//            MouseArea{
//                anchors.fill: parent
//                onClicked: scanner.start()
//            }

//            Text {
//                id: text1
//                anchors.centerIn: parent
//                text: qsTr("Text")
//                font.pixelSize: 12
//            }
//        }
        VideoPlayer{
            id: player
            width: 320
            height: 240
            z: 1
            visible: true
            anchors.centerIn: parent
        }

        Text {
            id: errorText
            x: 309
            y: 458
            font.pixelSize: 12
        }
    }


    Connections {
        id: scannerConnections
        target: scanner
        onNewImage: player.setImage(image)
    }

    Connections {
        target: scanner
        onErrorOccurred: errorText.text = error;
    }
}
