// ROS Instance Object
var ros;

// ROS Subscriber
var pico_sub;

// ROS Publisher
var command_pub;

// Text box reference
var pico_log;

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
});

function update_log(message) {
    var log = message.data;  // individual output
    var time = new Date().toTimeString().split(' ')[0];
    pico_log.text('[' + time + '] ' + log + pico_log.text());
}

function connect_rosbridge() {
    var address = "ws://" + $("#rosbridge_address").val();

    ros.connect(address);
}

window.onload = setup;