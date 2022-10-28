// ROS instance object
var ros;

// ROS subscriber
var pico_sub;

// ROS publisher
var command_pub;

// text box reference
var pico_log;

// plots
var alt_chart;
var temp_chart;
var press_chart;
var accel_chart;

// function to run on window load
function setup() {
    pico_log = $("#pico_log");  // jQuery get html element by id

    ros = new ROSLIB.Ros();  // ROS instance
    var rosbridge_status = $("#rosbridge_status");

    // ROS status functions
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
        pico_log.val('');  // clear pico log when server closes
    });

    // ROS topics
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

    // on corresponding button click, publish command to be read and executed by main.cpp
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

    $("#btn_data").click(() => {
        download();  // download data as csv function
    });

    Chart.defaults.global.defaultFontColor = 'black';

    // initialize altitude, temperature, pressure, and acceleration line graphs
    alt_chart = new Chart("altitude", {
        type: "line",
        data: {
            labels: [],  // x-values
            datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(0,0,255,1.0)",  // dot color: solid blue
            borderColor: "rgba(0,0,255,0.5)",  // line color: semi-transparent blue
            data: []  // y-values
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
            backgroundColor: "rgba(255,200,0,1.0)",  // yellow
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
            backgroundColor: "rgba(255,125,0,1.0)",  // orange
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
                backgroundColor: "rgba(0,200,0,1.0)",  // green
                borderColor: "rgba(0,200,0,0.5)",
                data: [],
                },{
                label: "Y",
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(200,0,0,1.0)",  // red
                borderColor: "rgba(200,0,0,0.5)",
                data: [],
                },{
                label: "Z",
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(0,0,200,1.0)",  // blue
                borderColor: "rgba(0,0,200,0.5)",
                data: [],
                }
            ]
        },
        options: {
            legend: {
                display: true,  // show dataset label for each color line
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

// declare and assign string variable for key pressed
var key_pressed = "none";


// on key release, turn off all motors and reset key_pressed
window.addEventListener('keyup', function(event) {
    const key = event.key.toUpperCase();
    if (key == key_pressed) {
        $("#btn_halt").click();
        key_pressed = "none";
    }
});

// on key pressed down, click appropriate button and set key_pressed
window.addEventListener('keydown', function(event) {
    // if key is pressed and held or a different key is already pressed, exit function
    if (event.repeat || key_pressed != "none") return;
    const key = event.key.toUpperCase();
    // movement controls
    if (key == 'W') {
        $("#btn_forward").click();
        key_pressed = 'W';
    }
    else if (key == 'S') {
        $("#btn_backward").click();
        key_pressed = 'S';
    }
    else if (key == 'A') {
        $("#btn_left").click();
        key_pressed = 'A';
    }
    else if (key == 'D') {
        $("#btn_right").click();
        key_pressed = 'D';
    }
    // arm controls
    else if (key == 'Q') {
        $("#btn_extend").click();
    }
    else if (key == 'E') {
        $("#btn_retract").click();
    } 
    else if (key == 'Z') {
        console.log("extend one");
        var command = new ROSLIB.Message({
            data:"extend_one"
        });
        command_pub.publish(command);
    } 
    else if (key == 'C') {
        console.log("retract one");
        var command = new ROSLIB.Message({
            data:"retract_one"
        });
        command_pub.publish(command);
    } 
    else if (key == 'R') {
        console.log("set position");
        var command = new ROSLIB.Message({
            data:"set_pos"
        });
        command_pub.publish(command);
    } 
    // backup stop motors key
    else if (key == 'H') {
        $("#btn_halt").click();
    }
});

var rows = []; // initialize array to store each row of data

function update_log(message) {
    var log = message.data;  // individual output
    var time = new Date().toTimeString().split(' ')[0];
    pico_log.val('[' + time + '] ' + log + pico_log.val());  // append to pico log
    var data_array = log.split(',');  // turn output separated by comma delimiter into array
    if (!isNaN(data_array[0][0])) {  // if the first character is a number
        let time = data_array[0];  // declare and assign each element to appropriate variable
        let alt = data_array[1];
        let temp = data_array[2];
        let press = data_array[3];
        let accel_x = data_array[4];
        let accel_y = data_array[5];
        let accel_z = data_array[6];
        addTimeData(time);  // add data entries to appropriate chart using functions defined below
        addData(alt_chart, alt);
        addData(temp_chart, temp);
        addData(press_chart,  press);
        addAccelData(accel_chart, 0, accel_x);
        addAccelData(accel_chart, 1, accel_y);
        addAccelData(accel_chart, 2, accel_z);

        rows.push(data_array);  // add data row entry to rows array
    }
}

// download data as csv
let csv_content = "data:text/csv;charset=utf-8,";

function download() {
    rows.forEach(function(row_array) {  // for each row_array in rows array
        let row = row_array.join(',');  // separate entries by comma delimiter
        csv_content += row;  // add to csv content
    });

    var encoded_uri = encodeURI(csv_content);
    var link = document.createElement('a');  // create hidden link to download content
    link.setAttribute("href", encoded_uri);
    link.setAttribute("download", "shc-twomonth-rover-data.csv");  // name of file to save as
    document.body.appendChild(link);

    link.click();  // click link when function is called
}

// connect to rosbridge address, editable on webpage
function connect_rosbridge() {
    var address = "ws://" + $("#rosbridge_address").val();
    ros.connect(address);
}

// add element to y-values
function addData(chart, data) {
    chart.data.datasets[0].data.push(data);
    chart.update();
}

// add time data to each chart's x-values
function addTimeData(time) {
    alt_chart.data.labels.push(time);
    temp_chart.data.labels.push(time);
    press_chart.data.labels.push(time);
    accel_chart.data.labels.push(time);
}

// add data to specified list of y-values in multiline plot
function addAccelData(chart, index, data) {
    chart.data.datasets[index].data.push(data);
    chart.update();
}

window.onload = setup;  // on window load, execute setup function
