// ROS Instance Object
var ros;

// ROS Subscriber
var pico_sub;

// ROS Publisher
var command_pub;

// Text box reference
var pico_log;

var alt_chart;
var temp_chart;
var press_chart;
var accel_chart;
var gryo_chart;

function setup() {
    pico_log = $("#pico_log");

    ros = new ROSLIB.Ros();
    var rosbridge_status = $("#rosbridge_status");
    ros.on('connection', function () {
        console.log('Connected to websocket server.');
        rosbridge_status.val("Connected");
        pico_log.val("Time (s), Altitude (m), Temperature (*C), Pressure (hPa), Accel X (m/s^2), Accel Y, Accel Z");
    });

    ros.on('error', function (error) {
        console.log('Error connection to websocket server: ' + error);
        rosbridge_status.val("Error");
    });

    ros.on('close', function () {
        console.log('Connection to websocket server closed.');
        rosbridge_status.val("Closed");
        pico_log.val('');
    });

    pico_sub = new ROSLIB.Topic({
        ros: ros,
        name: '/pico/output',
        messageType: 'std_msgs/String'
    });
    pico_sub.subscribe(update_log);

    command_pub = new ROSLIB.Topic({
        ros: ros,
        name: '/pico/command',
        messageType: 'std_msgs/String'
    });

    $("#rosbridge_connect").click(connect_rosbridge);

    /* $("#btn_forward").click(() => {
        console.log("forward");
        var command = new ROSLIB.Message({
            data:"forward"
        });
        command_pub.publish(command);
    });

    $("#btn_backward").click(() => {
        console.log("backward");
        var command = new ROSLIB.Message({
            data:"backward"
        });
        command_pub.publish(command);
    });

    $("#btn_left").click(() => {
        console.log("left");
        var command = new ROSLIB.Message({
            data:"left"
        });
        command_pub.publish(command);
    });

    $("#btn_right").click(() => {
        console.log("right");
        var command = new ROSLIB.Message({
            data:"right"
        });
        command_pub.publish(command);
    }); */
    
    $("#btn_extend").click(() => {
        console.log("extend");
        var command = new ROSLIB.Message({
            data:"extend"
        });
        command_pub.publish(command);
    });
        
    $("#btn_retract").click(() => {
        console.log("retract");
        var command = new ROSLIB.Message({
            data:"retract"
        });
        command_pub.publish(command);
    });

    $("#btn_halt").click(() => {
        console.log("halt");
        var command = new ROSLIB.Message({
            data:"halt"
        });
        command_pub.publish(command);
    });

    $("#btn_auto").click(() => {
        console.log("auto");
        var command = new ROSLIB.Message({
            data:"auto"
        });
        command_pub.publish(command);
    });

    $("#btn_LEDoff").click(() => {
        console.log("LED off");
        var command = new ROSLIB.Message({
            data:"LEDoff"
        });
        command_pub.publish(command);
    });

    $("#btn_data").click(() => {
        download();
    });

    Chart.defaults.global.defaultFontColor = 'black';

    alt_chart = new Chart("altitude", {
        type: "line",
        data: {
            labels: [],
            datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(0,0,255,1.0)",
            borderColor: "rgba(0,0,255,0.5)",
            data: []
            }]
        },
        options: {
            legend: {
                display: false
            },
            title: {
                display: true,
                text: "Altitude vs. Time"
            },
            scales: {
                xAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Time (s)"
                    }
                }],
                yAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Altitude (m)"
                    }
                }]
            }
        }
    });

    temp_chart = new Chart("temperature", {
        type: "line",
        data: {
            labels: [],
            datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(255,200,0,1.0)",
            borderColor: "rgba(255,200,0,0.5)",
            data: []
            }]
        },
        options: {
            legend: {
                display: false
            },
            title: {
                display: true,
                text: "Temperature vs. Time"
            },
            scales: {
                xAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Time (s)"
                    }
                }],
                yAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Temperature (*C)"
                    }
                }]
            }
        }
    });

    press_chart = new Chart("pressure", {
        type: "line",
        data: {
            labels: [],
            datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(255,125,0,1.0)",
            borderColor: "rgba(255,125,0,0.5)",
            data: []
            }]
        },
        options: {
            legend: {
                display: false
            },
            title: {
                display: true,
                text: "Pressure vs. Time"
            },
            scales: {
                xAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Time (s)"
                    }
                }],
                yAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Pressure (hPa)"
                    }
                }]
            }
        }
    });

    accel_chart = new Chart("acceleration", {
        type: "line",
        data: {
            labels: [],
            datasets: [{
                label: "X",
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(0,200,0,1.0)",
                borderColor: "rgba(0,200,0,0.5)",
                data: [],
                },{
                label: "Y",
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(200,0,0,1.0)",
                borderColor: "rgba(200,0,0,0.5)",
                data: [],
                },{
                label: "Z",
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(0,0,200,1.0)",
                borderColor: "rgba(0,0,200,0.5)",
                data: [],
                }
            ]
        },
        options: {
            legend: {
                display: true,
            },
            title: {
                display: true,
                text: "Acceleration vs. Time"
            },
            scales: {
                xAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Time (s)"
                    }
                }],
                yAxes: [{
                    scaleLabel: {
                        display: true,
                        labelString: "Acceleration (m/s^2)"
                    }
                }]
            }
        }
    });
}

window.addEventListener('keyup', function(event) {
     const key = event.key.toUpperCase();
     //event.preventDefault();
     if (key == 'W') {
        $("#btn_halt").click();
     }
     else if (key == 'S') {
        $("#btn_halt").click();
     }
     else if (key == 'A') {
        $("#btn_halt").click();
     }
     else if (key == 'D') {
        $("#btn_halt").click();
     }
     else if (key == 'Q') {
        $("#btn_halt").click();
     }
     else if (key == 'E') {
        $("#btn_halt").click();
     }
});

window.addEventListener('keydown', function(event) {
    const key = event.key.toUpperCase()
    //event.preventDefault();
    if (event.repeat) return;
    if (key == 'W') {
        $("#btn_forward").click();
    }
    else if (key == 'S') {
        $("#btn_backward").click();
    }
    else if (key == 'A') {
        $("#btn_left").click();
    }
    else if (key == 'D') {
        $("#btn_right").click();
    }
    else if (key == 'Q') {
        $("#btn_extend").click();
    }
    else if (key == 'E') {
        $("#btn_retract").click();
    } 
    else if (key == 'H') {
        $("#btn_halt").click();
    }
    // // temp random data generation
    // else if (key == 'P') {
    //     console.log("plot");
    //     var time = new Date().toTimeString().split(' ')[0];
    //     addData(alt_chart, Math.floor(Math.random()*6));
    //     addData(temp_chart, Math.floor(Math.random()*6));
    //     addData(press_chart, Math.floor(Math.random()*6));
    //     addTimeData(time);
    //     addAccelData(accel_chart, 0, Math.floor(Math.random()*6));
    //     addAccelData(accel_chart, 1, Math.floor(Math.random()*6));
    //     addAccelData(accel_chart, 2, Math.floor(Math.random()*6));
    //     pico_log.val('[' + time + '] ' + "random data\n" + pico_log.val());
    // }
    // else if (key == "ENTER") {  // temp manual data collection command
    //     console.log("data");
    //     var command = new ROSLIB.Message({
    //         data:"data"
    //     });
    //     command_pub.publish(command);
    // }
});

var rows = [];

let csv_content = "data:text/csv;charset=utf-8,";

function update_log(message) {
    var log = message.data;  // individual output
    var time = new Date().toTimeString().split(' ')[0];
    pico_log.val('[' + time + '] ' + log + pico_log.val());
    var data_array = log.split(',');
    if (!isNaN(data_array[0][0])) {
        let time = data_array[0];
        let alt = data_array[1];
        let temp = data_array[2];
        let press = data_array[3];
        let accel_x = data_array[4];
        let accel_y = data_array[5];
        let accel_z = data_array[6];
        addTimeData(time);
        addData(alt_chart, alt);
        addData(temp_chart, temp);
        addData(press_chart,  press);
        addAccelData(accel_chart, 0, accel_x);
        addAccelData(accel_chart, 1, accel_y);
        addAccelData(accel_chart, 2, accel_z);

        rows.push(data_array);
    }
}

// download data as csv
function download() {
    rows.forEach(function(row_array) {
        let row = row_array.join(',');
        csv_content += row;
    });

    var encoded_uri = encodeURI(csv_content);
    var link = document.createElement('a');
    link.setAttribute("href", encoded_uri);
    link.setAttribute("download", "shc-twomonth-rover-data.csv");
    document.body.appendChild(link);

    link.click();
}

function connect_rosbridge() {
    var address = "ws://" + $("#rosbridge_address").val();
    ros.connect(address);
}

function addData(chart, data) {
    chart.data.datasets[0].data.push(data);
    chart.update();
}

function addTimeData(time) {
    alt_chart.data.labels.push(time);
    temp_chart.data.labels.push(time);
    press_chart.data.labels.push(time);
    accel_chart.data.labels.push(time);
}

function addAccelData(chart, index, data) {
    chart.data.datasets[index].data.push(data);
    chart.update();
}

window.onload = setup;
