import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import VideoPlayer 1.0

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("Tighty Example")


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
                id: startButton
                text: "Start"
                anchors.left: parent.left
                enabled: scanner.isConnected && !scanner.isScanning;
                onClicked: scanner.start()
            }
            Button {
                text: "Replay"
                enabled: !startButton.enabled
                onClicked: {
                    fileDialog.visible = true;
                }
            }

            Button {
                text: "Record"
                //anchors.centerIn: parent
                enabled: false
            }

            Button {
                text: "Stop"
                enabled: scanner.isConnected && scanner.isScanning;
                onClicked: scanner.stop()
                anchors.right: parent.right
            }
        }

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


    FileDialog {
        id: fileDialog
        title: qsTr("Choose 3d real sense registration")
        folder: shortcuts.home
        selectMultiple: false
        onAccepted: scanner.playback(fileDialog.fileUrl)
        nameFilters: ["Real Sense registrations (*.bag)","All files (*)"]
        visible: false
    }

    Connections {
        id: scannerConnections
        target: scanner
        onNewImage: player.setImage(image)
    }

    Connections {
        target: scanner
        onErrorOccurred: errorText.text = error
    }
}
