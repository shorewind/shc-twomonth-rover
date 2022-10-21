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
        pico_log.text("Time (s), Temperature (*C), Pressure (hPa), Altitude (m), Accel X (m/s^2), Accel Y, Accel Z, Gyro X (rad/s), Gyro Y, Gyro Z");
    });

    ros.on('error', function (error) {
        console.log('Error connection to websocket server: ' + error);
        rosbridge_status.val("Error");
    });

    ros.on('close', function () {
        console.log('Connection to websocket server closed.');
        rosbridge_status.val("Closed");
        pico_log.text('');
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

    // Command Publishing
    // $("#btn_led_on").click(() => {
    //     var command = new ROSLIB.Message({
    //         data:"led_on"
    //     });
    //     command_pub.publish(command);
    // });

    // $("#btn_led_off").click(() => {
    //     var command = new ROSLIB.Message({
    //         data:"led_off"
    //     });
    //     command_pub.publish(command);
    // });

    // $("#btn_ping").click(() => {
    //     var command = new ROSLIB.Message({
    //         data:"ping"
    //     });
    //     command_pub.publish(command);
    // });
    
    // $("#btn_time").click(() => {
    //     var command = new ROSLIB.Message({
    //         data:"time"
    //     });
    //     command_pub.publish(command);
    // });

    $("#btn_forward").click(() => {
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
    });
    
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

    $("#btn_data").click(() => {
        console.log("data");
        var command = new ROSLIB.Message({
            data:"data"
        });
        command_pub.publish(command);
    });

    Chart.defaults.global.defaultFontColor = 'black';

    rot_chart = new Chart("rotation", {
        type: "line",
        data: {
            labels: [],
            datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(200,0,0,1.0)",
            borderColor: "rgba(200,0,0,0.5)",
            data: []
            }]
        },
        options: {
            legend: {
                display: false
            },
            title: {
                display: true,
                text: "Wheel Angular Velocity vs. Time"
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
                        labelString: "Angular Velocity (rad/s)"
                    }
                }]
            }
        }
    });

    dist_chart = new Chart("distance", {
        type: "line",
        data: {
            labels: [],
            datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(200,0,255,1.0)",
            borderColor: "rgba(200,0,255,0.5)",
            data: []
            }]
        },
        options: {
            legend: {
                display: false
            },
            title: {
                display: true,
                text: "Distance vs. Time"
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
                        labelString: "Distance (m)"
                    }
                }]
            }
        }
    });

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
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(0,200,0,1.0)",
                borderColor: "rgba(0,200,0,0.5)",
                data: [],
                }]
        },
        options: {
            legend: {
                display: false
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

window.addEventListener('keydown', function(event) {
    const key = event.key.toUpperCase()
    event.preventDefault();
    if (key == 'ENTER'){
        $("#btn_data").click();
    }
    else if (key == 'W'){
        $("#btn_forward").click();
    }
    else if (key == 'S'){
        $("#btn_backward").click();
    }
    else if (key == 'A'){
        $("#btn_left").click();
    }
    else if (key == 'D'){
        $("#btn_right").click();
    }
    else if (key == 'Q'){
        $("#btn_extend").click();
    }
    else if (key == 'E'){
        $("#btn_retract").click();
    }
    else if (key == 'P'){
        console.log("plot");
        var time = new Date().toTimeString().split(' ')[0];
        addData(alt_chart, time, Math.floor(Math.random()*6));
        addData(temp_chart, time, Math.floor(Math.random()*6));
        addData(press_chart, time, Math.floor(Math.random()*6));
        addData(accel_chart, time, Math.floor(Math.random()*6));
        addData(rot_chart, time, Math.floor(Math.random()*6));
        addData(dist_chart, time, Math.floor(Math.random()*6));
    }
});

var time_values = [];
var altitude_values = [];

function update_log(message) {
    var log = message.data;  // individual output
    var time = new Date().toTimeString().split(' ')[0];
    pico_log.text('[' + time + '] ' + log + pico_log.text());
    var data_array = log.split(',');
    if (!isNaN(data_array[0][0])) {
        time_values.push(data_array[0]);
        altitude_values.push(data_array[1]);
        addData(alt_chart, data_array[0], data_array[1]);
    }
}

function connect_rosbridge() {
    var address = "ws://" + $("#rosbridge_address").val();
    ros.connect(address);
}

function plot() {
    alt_chart = new Chart("altitude", {
    type: "line",
    data: {
        labels: [],
        datasets: [{
        fill: false,
        lineTension: 0,
        backgroundColor: "rgba(0,0,255,1.0)",
        borderColor: "rgba(0,0,255,0.1)",
        data: []
        }]
    },
    options: {
        legend: {display: false},
        scales: {
        yAxes: [{ticks: {min: 6, max:16}}],
        }
    }
    });
}

function addData(chart, label, data) {
    chart.data.labels.push(label);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(data);
    });
    chart.update();
}

function removeData(chart) {
    chart.data.labels.pop();
    chart.data.datasets.forEach((dataset) => {
        dataset.data.pop();
    });
    chart.update();
}

window.onload = setup;
// window.onload = plot;
