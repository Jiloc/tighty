import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import VideoPlayer 1.0

Window {
    visible: true
    width: 1280
    height: 580
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
                text: "Load File..."
                enabled: !scanner.isStreaming
                onClicked: fileDialog.visible = true
            }

            Button {
                text: "Start"
                enabled: scanner.isConnected && !scanner.isStreaming
                onClicked: scanner.start()
            }

            Button {
                text: "Record"
                enabled: scanner.isStreaming && !scanner.isScanning
                onClicked: scanner.record()
            }

            Button {
                text: "Stop"
                enabled: scanner.isStreaming
                onClicked: scanner.stop()
            }
        }

        VideoPlayer{
            id: rawStream
            width: 640
            height: 480
            z: 1
            visible: true
            anchors.left: parent.left
            anchors.bottom: parent.bottom
        }

        VideoPlayer{
            id: filteredStream
            width: 640
            height: 480
            z: 1
            visible: true
            anchors.right: parent.right
            anchors.bottom: parent.bottom
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
        onNewImage: rawStream.setImage(image)
        onNewProcessedImage: filteredStream.setImage(image)
        onErrorOccurred: errorText.text = error
    }
}
