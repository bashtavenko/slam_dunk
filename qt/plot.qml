import QtQuick 2.0
import QtCharts 2.0

ChartView {
    id: chart
    width: 400
    height: 300

    title: "My Chart"

    LineSeries {
        name: "Data"
        XYPoint { x: 0; y: 0 }
        XYPoint { x: 1; y: 1 }
    }
}