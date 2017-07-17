$(document).ready(principal);
var ip = '192.168.100.1';
//var ip = 'localhost';
var cam1 = 'usb_cam1', quality1 = '20', width1 = '640', height1 = '480';
var cam2 = 'usb_cam2', quality2 = '20', width2 = '640', height2 = '480';
var cam3 = 'usb_cam3', quality3 = '20', width3 = '640', height3 = '480';
var cam4 = 'usb_cam4', quality4 = '20', width4 = '640', height4 = '480';
var cam5 = 'usb_cam5', quality5 = '20', width5 = '640', height5 = '480';

var cam1Alive = false;
var cam2Alive = false;
var cam3Alive = false;
var cam4Alive = false;
var cam5Alive = false;

var t0CPattern = 0;
var t0Labels = 0;

var DRUM_TEXTURE = "https://keithclark.co.uk/labs/css-fps/drum2.png";
var led1Publish;
var led2Publish;

var flipper1_reset = 0;
var flipper2_reset = 0;
var flipper3_reset = 0;
var flipper4_reset = 0;

var flipper1_lec = 0.0;
var flipper2_lec = 0.0;
var flipper3_lec = 0.0;
var flipper4_lec = 0.0;

var flipper1_color = "white";
var flipper2_color = "white";
var flipper3_color = "white";
var flipper4_color = "white";


var base_start_reset = 0;
var shoulder_start_reset = 0;
var elbow_start_reset = 0;
var roll_start_reset = 0;
var pitch_start_reset = 0;

var base_end_reset = 0;
var shoulder_end_reset = 0;
var elbow_end_reset = 0;
var roll_end_reset = 0;
var pitch_end_reset = 0;

var base_lec = 0;
var shoulder_lec = 0;
var elbow_lec = 0;
var roll_lec = 0;
var pitch_lec = 0;

var base_color = "white";
var shoulder_color = "white";
var elbow_color = "white";
var roll_color = "white";
var pitch_color = "white";

var nodeStatus1_color = "white";
var nodeStatus2_color = "red";
var nodeStatus3_color = "white";
var nodeStatus4_color = "yellow";
var nodeStatus5_color = "white";
var nodeStatus6_color = "white";
var nodeStatus7_color = "orange";
var nodeStatus8_color = "white";
var nodeStatus9_color = "white";
var nodeStatus10_color = "white";
var nodeStatus11_color = "white";
var nodeStatus12_color = "white";
var nodeStatus13_color = "green";
var nodeStatus14_color = "white";
var nodeStatus15_color = "white";



function principal(){
	//-----------ROS Connection
	// Connect to ROS.
	var urlROS = 'ws://'+ip+':9090'
	ros = new ROSLIB.Ros({
	url : urlROS
	});

	ros.on('connection', function() {
	console.log('Connected to websocket server.');
	});

	ros.on('error', function(error) {
	console.log('Error connecting to websocket server: ', error);
	});

	ros.on('close', function() {
	console.log('Connection to websocket server closed.');
	});
	// Quality and Resolution
	$('input').eq(0).on('change',setGeneralQuality);
	$('input').eq(1).on('change',setQuality1);
	$('input').eq(2).on('change',setResolution1);
	$('input').eq(3).on('change',setQuality2);
	$('input').eq(4).on('change',setResolution2);
	$('input').eq(5).on('change',setQuality3);
	$('input').eq(6).on('change',setResolution3);
	$('input').eq(7).on('change',setLed1);
	$('input').eq(8).on('change',setLed2);
	$('input').eq(9).on('change',setQuality4);
	$('input').eq(10).on('change',setResolution4);
	$('input').eq(11).on('change',setTilt);
	$('input').eq(12).on('change',setQuality5);
	$('input').eq(13).on('change',setResolution5);
	$('input').eq(14).on('change',setPan);

	// Set cams
	$('select').eq(0).on('click', setCam1);
	$('select').eq(1).on('click', setCam2);
	$('select').eq(2).on('click', setCam3);
	$('select').eq(3).on('click', setCam4);
	$('select').eq(4).on('click', setCam5);

	$('.img-responsive').eq(0).attr('src', "http://"+ip+":8080/stream?topic=/"+cam2+"/image_raw&type=mjpeg&width="+width2+"&height="+height2+"&quality="+quality2);
	$('.img-responsive').eq(1).attr('src', "http://"+ip+":8080/stream?topic=/"+cam1+"/image_raw&type=mjpeg&width="+width1+"&height="+height1+"&quality="+quality1);
	$('.img-responsive').eq(2).attr('src', "http://"+ip+":8080/stream?topic=/"+cam3+"/image_raw&type=mjpeg&width="+width3+"&height="+height3+"&quality="+quality3);
	$('.img-responsive').eq(3).attr('src', "http://"+ip+":8080/stream?topic=/"+cam4+"/image_raw&type=mjpeg&width="+width4+"&height="+height4+"&quality="+quality4);
	$('.img-responsive').eq(4).attr('src', "http://"+ip+":8080/stream?topic=/"+cam5+"/image_raw&type=mjpeg&width="+width5+"&height="+height5+"&quality="+quality5);


	setPosTable();
	setStatusTable();

	//Publish Topics
	//--------------------------------
  	led1Publish = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/set/led_a',
		messageType : 'std_msgs/Int16'
	});

	led2Publish = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/set/led_b',
		messageType : 'std_msgs/Int16'
	});

	tiltPublish = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/set/camTilt',
		messageType : 'std_msgs/Int16'
	});
	panPublish = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/set/camPan',
		messageType : 'std_msgs/Int16'
	});

	tagsPublish = new ROSLIB.Topic({
		ros : ros,
		name : '/add_shape',
		messageType : 'std_msgs/String'
	});


	led1Publish.publish(
		new ROSLIB.Message({
			data : 0
		}) 
	);
	led2Publish.publish(
		new ROSLIB.Message({
			data : 0
		}) 
	);

	tiltPublish.publish(
		new ROSLIB.Message({
			data : 90
		})
	);

	panPublish.publish(
		new ROSLIB.Message({
			data : 90
		})
	);




	// Subscribe Topics
	// -----------------------
	var batteryListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/robotBattery',
		messageType : 'std_msgs/Int64'
	});

	var co2Listener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/sensors/co2',
		messageType : 'std_msgs/Int32'
	});

	var laptopBatteryListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/laptopBattery',
		messageType : 'std_msgs/Int32'
	});

	var thermalListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/sensors/thermal',
		messageType : 'std_msgs/Float32MultiArray'
	});

	var qrListener = new ROSLIB.Topic({
		ros : ros,
		name : '/markers',
		messageType : 'zbar_detector/Marker'
	});

	var imageQRListener = new ROSLIB.Topic({
		ros : ros,
		name : '/output/QRcodeResult/compressed',
    	messageType : 'sensor_msgs/CompressedImage'
	});

	var thermalStatusListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/thermalStatus',
		messageType : 'std_msgs/String'
	});

	var movementStatusListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/movementStatus',
		messageType : 'std_msgs/String'
	});	

	var co2StatusListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/co2Status',
		messageType : 'std_msgs/String'
	});

	var signalStatusListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/signalStatus',
		messageType : 'std_msgs/String'
	});

	var faceStatusListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/faceStatus',
		messageType : 'std_msgs/String'
	});

	var victimStatusListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/victimStatus',
		messageType : 'std_msgs/String'
	});


	var flagLabelsHazmateListener = new ROSLIB.Topic({
		ros : ros,
		name: '/flag_labels_hazmate',
		messageType : 'vision_msjs/labelDetect'
	});

	var flagCPaternListener = new ROSLIB.Topic({
		ros : ros,
		name: '/flag_c_pattern',
		messageType : 'vision_msjs/cPatternDetect'
	});


	//// Flippers -------------------------------------
	var flipLFResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'flipper1_reset',
		messageType : 'std_msgs/Int16'
	});

	var flipLBResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'flipper2_reset',
		messageType : 'std_msgs/Int16'
	});

	var flipRFResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'flipper3_reset',
		messageType : 'std_msgs/Int16'
	});

	var flipRBResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'flipper4_reset',
		messageType : 'std_msgs/Int16'
	});





	// Base --------------------------------------------
	var baseStartResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'base1_reset',
		messageType : 'std_msgs/Int16'
	});

	var baseEndResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'base2_reset',
		messageType : 'std_msgs/Int16'
	});

	// shoulder --------------------------------------------
	var shoulderStartResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'shoulder1_reset',
		messageType : 'std_msgs/Int16'
	});

	var shoulderEndResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'shoulder2_reset',
		messageType : 'std_msgs/Int16'
	});

	// elbow --------------------------------------------
	var elbowStartResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'elbow1_reset',
		messageType : 'std_msgs/Int16'
	});

	var elbowEndResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'elbow2_reset',
		messageType : 'std_msgs/Int16'
	});

	// roll --------------------------------------------
	var rollStartResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'roll1_reset',
		messageType : 'std_msgs/Int16'
	});

	var rollEndResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'roll2_reset',
		messageType : 'std_msgs/Int16'
	});

	// pitch --------------------------------------------
	var pitchStartResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'pitch1_reset',
		messageType : 'std_msgs/Int16'
	});

	var pitchEndResetListener = new ROSLIB.Topic({
		ros :ros,
		name : 'pitch2_reset',
		messageType : 'std_msgs/Int16'
	});



	var flipLFPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'flipper1_lec',
		messageType : 'std_msgs/Float32'
	});

	var flipLBPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'flipper2_lec',
		messageType : 'std_msgs/Float32'
	});

	var flipRFPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'flipper3_lec',
		messageType : 'std_msgs/Float32'
	});

	var flipRBPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'flipper4_lec',
		messageType : 'std_msgs/Float32'
	});

	
	var basePosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'base_lec',
		messageType : 'std_msgs/Float32'
	});
	var shoulderPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'shoulder_lec',
		messageType : 'std_msgs/Float32'
	});
	var elbowPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'elbow_lec',
		messageType : 'std_msgs/Float32'
	});	
	var rollPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'roll_lec',
		messageType : 'std_msgs/Float32'
	});	
	var pitchPosListener = new ROSLIB.Topic({
		ros : ros,
		name : 'pitch_lec',
		messageType : 'std_msgs/Float32'
	});	

	var cam1InfoListener = new ROSLIB.Topic({
		ros : ros,
		name : '/usb_cam1/camera_info',
		messageType : 'sensor_msgs/CameraInfo'
	});

	var cam2InfoListener = new ROSLIB.Topic({
		ros : ros,
		name : '/usb_cam2/camera_info',
		messageType : 'sensor_msgs/CameraInfo'
	});

	var cam3InfoListener = new ROSLIB.Topic({
		ros : ros,
		name : '/usb_cam3/camera_info',
		messageType : 'sensor_msgs/CameraInfo'
	});

	var cam4InfoListener = new ROSLIB.Topic({
		ros : ros,
		name : '/usb_cam4/camera_info',
		messageType : 'sensor_msgs/CameraInfo'
	});

	var cam5InfoListener = new ROSLIB.Topic({
		ros : ros,
		name : '/usb_cam5/camera_info',
		messageType : 'sensor_msgs/CameraInfo'
	});


	//// Service

	//img_labels_result
	var imgLabelsResultClient = new ROSLIB.Service({
		ros : ros,
		name : '/img_labels_result',
		serviceType : 'vision_msjs/imgLabels'
	});

	var requestImgLabel = new ROSLIB.ServiceRequest({

	});

	var imgCPatternResultClient = new ROSLIB.Service({
		ros : ros,
		name : '/img_cpattern_result',
		serviceType : 'vision_msjs/imgCpattern'
	});

	var requestImgCPattern = new ROSLIB.ServiceRequest({

	});	



	batteryListener.subscribe(function(message){
		var levelRobotBaterry = message.data;
		console.log("Level battery:"+levelRobotBaterry);
		if($('.progress-bar').eq(0).hasClass('progress-bar-success')){
		  $('.progress-bar').eq(0).toggleClass('progress-bar-success');
		}
		if($('.progress-bar').eq(0).hasClass('progress-bar-danger')){
		  $('.progress-bar').eq(0).toggleClass('progress-bar-danger');
		}
		var percent = Math.round(10*(levelRobotBaterry-100)/4);
		$('.progress-bar').eq(0).css("width", percent + "%");
		$('.progress-bar').eq(0).html('<strong>' + percent + '% - '+(levelRobotBaterry/10) +' V</strong>');
		if(percent>80.0){
		  $('.progress-bar').eq(0).toggleClass('progress-bar-success');
		}
		else if(percent>45.0){
		}
		else{
		  $('.progress-bar').eq(0).toggleClass('progress-bar-danger');
		}
	});

	co2Listener.subscribe(function(message){
		var levelCo2 = message.data/10;
		console.log("CO2:"+levelCo2);
		if($('.progress-bar').eq(2).hasClass('progress-bar-success')){
		  $('.progress-bar').eq(2).toggleClass('progress-bar-success');
		}
		if($('.progress-bar').eq(2).hasClass('progress-bar-danger')){
		  $('.progress-bar').eq(2).toggleClass('progress-bar-danger');
		}
		var percent = Math.round(100*(levelCo2/100));
		$('.progress-bar').eq(2).css("width", percent + "%");
		$('.progress-bar').eq(2).html('<strong>' + percent + '%' +'</strong>');
		if(percent>80.0){
		  $('.progress-bar').eq(2).toggleClass('progress-bar-success');
		}
		else if(percent>45.0){
		}
		else{
		  $('.progress-bar').eq(2).toggleClass('progress-bar-danger');
		}

	});

	laptopBatteryListener.subscribe(function(message){
		var levelLaptopBaterry = message.data;

		if($('.progress-bar').eq(1).hasClass('progress-bar-success')){
		  $('.progress-bar').eq(1).toggleClass('progress-bar-success');
		}
		if($('.progress-bar').eq(1).hasClass('progress-bar-danger')){
		  $('.progress-bar').eq(1).toggleClass('progress-bar-danger');
		}
		var percent = Math.round(100*(levelLaptopBaterry/100));
		$('.progress-bar').eq(1).css("width", percent + "%");
		$('.progress-bar').eq(1).html('<strong>' + percent + '%' +'</strong>');
		if(percent>80.0){
		  $('.progress-bar').eq(1).toggleClass('progress-bar-success');
		}
		else if(percent>45.0){
		}
		else{
		  $('.progress-bar').eq(1).toggleClass('progress-bar-danger');
		}

	});

	thermalListener.subscribe(function(message){
		var thermalData = message.data;
		var cadena = "<table ><tr>";
		var colores =["#040530","#0e127c","#1a41bf","#1a72bf","#22bee5","#e9ed28","#edc528","#ed8a28","#c6441f","#c61f1f"];
		var temp = 0;
		for (i=0;i<thermalData.length;i++){
			temp= Math.round((thermalData[i]+150)/29);
			//console.log("Dato:"+temp);
			cadena+="<td  bgcolor="+colores[temp]+"></td>"
			if( (i+1)%16 == 0 ){
				cadena+="</tr><tr>";
			}
		}
		cadena+="</tr></table>"

		$('#TERM').html(cadena);
	});

	qrListener.subscribe(function(message){
		var qrData = message.data;
		console.log("markers:"+qrData);

		var cadena = qrData;


		$('#statusQR').html(cadena);
	});

	imageQRListener.subscribe(function(message){
		console.log("hay imagen");
		var ImageData1="data:image/jpeg;base64,"+message.data;
		displayImage = document.getElementById("imageDetected");
		displayImage.setAttribute('src', ImageData1);
	});

	thermalStatusListener.subscribe(function(message){
		$('#thermalStatus').html(message.data);
	});

	movementStatusListener.subscribe(function(message){
		$('#movementStatus').html(message.data);
	});

	co2StatusListener.subscribe(function(message){
		$('#co2Status').html(message.data);
	});

	signalStatusListener.subscribe(function(message){
		$('#signalStatus').html(message.data);
	});

	faceStatusListener.subscribe(function(message){
		$('#faceStatus').html(message.data);
	});

	victimStatusListener.subscribe(function(message){
		$('#victimStatus').html(message.data);
	});

	flagLabelsHazmateListener.subscribe(function(message){
		
		imgLabelsResultClient.callService(requestImgLabel,function(result){
			var labels ="";
			if(result.label1!="nothing"){
				labels+=result.label1+" ";
			}
			if(result.label2!="nothing"){
				labels+=result.label2+" ";
			}
			if(result.label3!="nothing"){
				labels+=result.label3+" ";
			}
			if(result.label4!="nothing"){
				labels+=result.label4+" ";
			}
			
			console.log("Imagen recibida de "+imgLabelsResultClient.name);
			$('#signalStatus').html(labels);
			displayImage = document.getElementById("imageDetected");
			displayImage.setAttribute('src',"/roswww/img/labelDetected"+result.label1+result.label2+result.label3+result.label4+".jpg");

			if ((new Date().getTime() - t0Labels) > 2000){
				var res = "Labels: " + labels;
				tagsPublish.publish(
					new ROSLIB.Message({
						data : res
					})
				);
				t0Labels = new Date().getTime();
			}

		});
	});

	flagCPaternListener.subscribe(function(message){

		imgCPatternResultClient.callService(requestImgCPattern,function(result){
			$('#signalStatus').html("Gap angle: "+result.gap_angle+" rad");
			displayImage = document.getElementById("imageDetected");
			displayImage.setAttribute('src',"/roswww/img/cPatternDetected"+result.gap_angle+".jpg");
			

			if ((new Date().getTime() - t0CPattern) > 2000){
				var res = "angle " + result.gap_angle+" rad";
				tagsPublish.publish(
					new ROSLIB.Message({
						data : res
					})
				);
				t0CPattern = new Date().getTime();
			}

			

		});

	});
	
//panPublish.publish(
//		new ROSLIB.Message({
//			data : 90
//		})
//	);
	

	// Service
	// ----------------------

//	var addTwoIntsClient = new ROSLIB.Service({
//		ros : ros,
//		name : '/add_two_ints',
//		serviceType : 'rospy_tutorials/AddTwoInts'
//	});
//
//	var request = new ROSLIB.ServiceRequest({
//		a : 1,
//		b : 2
//	});
//
//	addTwoIntsClient.callService(request,function(result){
//		console.log('Result for service call on'
//			+ addTwoIntsClient.name
//			+ ': '
//			+ result.sum);
//	});
	

	// flipper 1 ---------------------------------------
	flipLFResetListener.subscribe(function(message){
		console.log("Data:"+message.data);
		if(message.data==1){
			flipper1_color="red";
		}else{
			flipper1_color="white";
		}
		setPosTable();
	});

	// flipper 2 ---------------------------------------
	flipLBResetListener.subscribe(function(message){
		console.log("Data:"+message.data);
		if(message.data==1){
			flipper2_color="red";
		}else{
			flipper2_color="white";
		}
		setPosTable();
	});

	// flipper 3 ---------------------------------------
	flipRFResetListener.subscribe(function(message){
		console.log("Data:"+message.data);
		if(message.data==1){
			flipper3_color="red";
		}else{
			flipper3_color="white";
		}
		setPosTable();
	});

	// flipper 4 ---------------------------------------
	flipRBResetListener.subscribe(function(message){
		console.log("Data:"+message.data);
		if(message.data==1){
			flipper4_color="red";
		}else{
			flipper4_color="white";
		}
		setPosTable();
	});

	// base ---------------------------------------
	baseStartResetListener.subscribe(function(message){
		base_start_reset = message.data;
		if(base_start_reset==1){
			base_color="red";
		}
		if(base_start_reset==0 && base_end_reset==0){
			base_color="white";
		}
		setPosTable();
	});
	baseEndResetListener.subscribe(function(message){
		base_end_reset = message.data;
		if(base_end_reset==1){
			base_color="orange";
		}
		if(base_start_reset==0 && base_end_reset==0){
			base_color="white";
		}
		setPosTable();
	});

	// shoulder ---------------------------------------
	shoulderStartResetListener.subscribe(function(message){
		shoulder_start_reset = message.data;
		if(shoulder_start_reset==1){
			shoulder_color="red";
		}
		if(shoulder_start_reset==0 && shoulder_end_reset==0){
			shoulder_color="white";
		}
		setPosTable();
	});
	shoulderEndResetListener.subscribe(function(message){
		shoulder_end_reset = message.data;
		if(shoulder_end_reset==1){
			shoulder_color="orange";
		}
		if(shoulder_start_reset==0 && shoulder_end_reset==0){
			shoulder_color="white";
		}
		setPosTable();
	});

	// elbow ---------------------------------------
	elbowStartResetListener.subscribe(function(message){
		elbow_start_reset = message.data;
		if(elbow_start_reset==1){
			elbow_color="red";
		}
		if(elbow_start_reset==0 && elbow_end_reset==0){
			elbow_color="white";
		}
		setPosTable();
	});
	elbowEndResetListener.subscribe(function(message){
		elbow_end_reset = message.data;
		if(elbow_end_reset==1){
			elbow_color="orange";
		}
		if(elbow_start_reset==0 && elbow_end_reset==0){
			elbow_color="white";
		}
		setPosTable();
	});

	// roll ---------------------------------------
	rollStartResetListener.subscribe(function(message){
		roll_start_reset = message.data;
		if(roll_start_reset==1){
			roll_color="red";
		}
		if(roll_start_reset==0 && roll_end_reset==0){
			roll_color="white";
		}
		setPosTable();
	});
	rollEndResetListener.subscribe(function(message){
		roll_end_reset = message.data;
		if(roll_end_reset==1){
			roll_color="orange";
		}
		if(roll_start_reset==0 && roll_end_reset==0){
			roll_color="white";
		}
		setPosTable();
	});

	// pitch ---------------------------------------
	pitchStartResetListener.subscribe(function(message){
		pitch_start_reset = message.data;
		if(pitch_start_reset==1){
			pitch_color="red";
		}
		if(pitch_start_reset==0 && pitch_end_reset==0){
			pitch_color="white";
		}
		setPosTable();
	});
	pitchEndResetListener.subscribe(function(message){
		pitch_end_reset = message.data;
		if(pitch_end_reset==1){
			pitch_color="orange";
		}
		if(pitch_start_reset==0 && pitch_end_reset==0){
			pitch_color="white";
		}
		setPosTable();
	});



	// Flipper pos
	flipLFPosListener.subscribe(function(message){
		flipper1_lec = Math.round(message.data);
		setPosTable();
	});
	flipLBPosListener.subscribe(function(message){
		flipper2_lec = Math.round(message.data);
		setPosTable();
	});
	flipRFPosListener.subscribe(function(message){
		flipper3_lec = Math.round(message.data);
		setPosTable();
	});
	flipRBPosListener.subscribe(function(message){
		flipper4_lec = Math.round(message.data);
		setPosTable();
	});

	basePosListener.subscribe(function(message){
		base_lec = Math.round(message.data);
		setPosTable();
	});
	shoulderPosListener.subscribe(function(message){
		shoulder_lec = Math.round(message.data);
		setPosTable();
	});
	elbowPosListener.subscribe(function(message){
		elbow_lec = Math.round(message.data);
		setPosTable();
	});
	rollPosListener.subscribe(function(message){
		roll_lec = Math.round(message.data);
		setPosTable();
	});	
	pitchPosListener.subscribe(function(message){
		pitch_lec = Math.round(message.data);
		setPosTable();
	});

	cam1InfoListener.subscribe(function(message){
		cam1Alive = true;
	});

	cam2InfoListener.subscribe(function(message){
		cam2Alive = true;
	});

	cam3InfoListener.subscribe(function(message){
		cam3Alive = true;
	});

	cam4InfoListener.subscribe(function(message){
		cam4Alive = true;
	});

	cam5InfoListener.subscribe(function(message){
		cam5Alive = true;
	});


	Concurrent.Thread.create(function(){
		sleep(3000);
		if(cam1Alive==false)
			$('.img-responsive').eq(1).attr('src', "img/camera.jpg");
		if(cam2Alive==false)
			$('.img-responsive').eq(0).attr('src', "img/camera.jpg");
		if(cam3Alive==false)
			$('.img-responsive').eq(2).attr('src', "img/camera.jpg");
		if(cam4Alive==false)
			$('.img-responsive').eq(3).attr('src', "img/camera.jpg");
		if(cam5Alive==false)
			$('.img-responsive').eq(4).attr('src', "img/camera.jpg");
    	var i = 0;
    	while ( 1 ) {
    		
    		$('#victimStatus').html("Count: "+i);
       		i++;
       		sleep(1000);

    	}
	});


}

function sleep(milliseconds) {
  var start = new Date().getTime();
  for (var i = 0; i < 1e7; i++) {
    if ((new Date().getTime() - start) > milliseconds){
      break;
    }
  }
}

function getDatetime(){
	var currentdate = new Date(); 
	var datetime = currentdate.getFullYear()+"-"
		+(currentdate.getMonth()+1)+"-"
		+currentdate.getDate() + " "
        + currentdate.getHours() + ":"  
        + currentdate.getMinutes() + ":" 
        + currentdate.getSeconds();
    return datetime;
}

//http://192.168.100.239:8080/stream?topic=/usb_cam2/image_raw&type=mjpeg&quality=20
//http:192.168.0.50:8080/stream?topic=/usb_cam2/image_raw&type=mjpeg&width=640&height=480&quality=20
function setCam1(){
	cam2 = $(this).val();
	console.log("CAM2:_" + cam2+"_");
	if (cam2=="empty") {
		$('.img-responsive').eq(0).attr('src', "img/camera.jpg");

	}else{
		var src = 'http://' + ip + ':8080/stream?topic=/' + cam2 + '/image_raw&type=mjpeg&width=' + width1 +'&height=' + height1 + '&quality=' + quality1;
		$('.img-responsive').eq(0).attr('src', src);
	}

	
}

function setCam2(){
	cam1 = $(this).val();
	console.log("CAM1:_" + cam1+"_");
	if (cam1=="empty") {
		$('.img-responsive').eq(1).attr('src', "img/camera.jpg");
	}else{
		var src = 'http://' + ip + ':8080/stream?topic=/' + cam1 + '/image_raw&type=mjpeg&width=' + width2 +'&height=' + height2 + '&quality=' + quality2;
		$('.img-responsive').eq(1).attr('src', src);
	}
}

function setCam3(){
	cam3 = $(this).val();

	if (cam3=="empty") {
		$('.img-responsive').eq(2).attr('src', "img/camera.jpg");
	}else{
		var src = 'http://' + ip + ':8080/stream?topic=/' + cam3 + '/image_raw&type=mjpeg&width=' + width3 +'&height=' + height3 + '&quality=' + quality3;
		$('.img-responsive').eq(2).attr('src', src);
	}
}

function setCam4(){
	cam4 = $(this).val();
	if (cam4=="empty") {
		$('.img-responsive').eq(3).attr('src', "img/camera.jpg");
	}else{
		var src = 'http://' + ip + ':8080/stream?topic=/' + cam4 + '/image_raw&type=mjpeg&width=' + width4 +'&height=' + height4 + '&quality=' + quality4;
		$('.img-responsive').eq(3).attr('src', src);
	}
}

function setCam5(){
	cam5 = $(this).val();
	if (cam5=="empty") {
		$('.img-responsive').eq(4).attr('src', "img/camera.jpg");
	}else{
		var src = 'http://' + ip + ':8080/stream?topic=/' + cam5 + '/image_raw&type=mjpeg&width=' + width5 +'&height=' + height5 + '&quality=' + quality5;
		$('.img-responsive').eq(4).attr('src', src);
	}
}


function setGeneralQuality(){
	quality2 = $(this).val().toString();
	quality3 = $(this).val().toString();
	quality4 = $(this).val().toString();
	quality5 = $(this).val().toString();
	var src2 = 'http://' + ip + ':8080/stream?topic=/' + cam2 + '/image_raw&type=mjpeg&width=' + width2 +'&height=' + height2 + '&quality=' + quality2;
	var src3 = 'http://' + ip + ':8080/stream?topic=/' + cam3 + '/image_raw&type=mjpeg&width=' + width3 +'&height=' + height3 + '&quality=' + quality3;
	var src4 = 'http://' + ip + ':8080/stream?topic=/' + cam4 + '/image_raw&type=mjpeg&width=' + width4 +'&height=' + height4 + '&quality=' + quality4;
	var src5 = 'http://' + ip + ':8080/stream?topic=/' + cam5 + '/image_raw&type=mjpeg&width=' + width5 +'&height=' + height5 + '&quality=' + quality5;
	$('.img-responsive').eq(0).attr('src', src2);
	$('.img-responsive').eq(2).attr('src', src3);
	$('.img-responsive').eq(3).attr('src', src4);
	$('.img-responsive').eq(4).attr('src', src5);

	$('#QG').html($(this).val() + ' %');
	$('#Q1').html($(this).val() + ' %');
	$('#Q3').html($(this).val() + ' %');
	$('#Q4').html($(this).val() + ' %');
	$('#Q5').html($(this).val() + ' %');


	

}

function setQuality1(){
	quality2 = $(this).val().toString();
	//console.log("Quality: " + quality1);
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam2 + '/image_raw&type=mjpeg&width=' + width2 +'&height=' + height2 + '&quality=' + quality2;
	$('.img-responsive').eq(0).attr('src', src);
	$('#Q1').html($(this).val() + ' %');
}
function setQuality2(){
	quality1 = $(this).val().toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam1 + '/image_raw&type=mjpeg&width=' + width1 +'&height=' + height1 + '&quality=' + quality1;
	$('.img-responsive').eq(1).attr('src', src);
	$('#Q2').html($(this).val() + ' %');
}
function setQuality3(){
	quality3 = $(this).val().toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam3 + '/image_raw&type=mjpeg&width=' + width3 +'&height=' + height3 + '&quality=' + quality3;
	$('.img-responsive').eq(2).attr('src', src);
	$('#Q3').html($(this).val() + ' %');
}
function setQuality4(){
	quality4 = $(this).val().toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam4 + '/image_raw&type=mjpeg&width=' + width4 +'&height=' + height4 + '&quality=' + quality4;
	$('.img-responsive').eq(3).attr('src', src);
	$('#Q4').html($(this).val() + ' %');
}
function setQuality5(){
	quality5 = $(this).val().toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam5 + '/image_raw&type=mjpeg&width=' + width5 +'&height=' + height5 + '&quality=' + quality5;
	$('.img-responsive').eq(4).attr('src', src);
	$('#Q5').html($(this).val() + ' %');
}





function setResolution1(){
	var size = $(this).val();
	width2  = Math.round(160 + 480 * size / 100).toString();
	height2 = Math.round(120 + 360 * size / 100).toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam2 + '/image_raw&type=mjpeg&width=' + width2 +'&height=' + height2 + '&quality=' + quality2;
	$('.img-responsive').eq(0).attr('src', src);
	$('#R1').html(width1+' x '+height1);
}
function setResolution2(){
	var size = $(this).val();
	width1  = Math.round(160 + 480 * size / 100).toString();
	height1 = Math.round(120 + 360 * size / 100).toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam1 + '/image_raw&type=mjpeg&width=' + width1 +'&height=' + height1 + '&quality=' + quality1;
	$('.img-responsive').eq(1).attr('src', src);
	$('#R2').html(width2+' x '+height2);
}
function setResolution3(){
	var size = $(this).val();
	width3  = Math.round(160 + 480 * size / 100).toString();
	height3 = Math.round(120 + 360 * size / 100).toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam3 + '/image_raw&type=mjpeg&width=' + width3 +'&height=' + height3 + '&quality=' + quality3;
	$('.img-responsive').eq(2).attr('src', src);
	$('#R3').html(width3+' x '+height3);
}
function setResolution4(){
	var size = $(this).val();
	width4  = Math.round(160 + 480 * size / 100).toString();
	height4 = Math.round(120 + 360 * size / 100).toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam4 + '/image_raw&type=mjpeg&width=' + width4 +'&height=' + height4 + '&quality=' + quality4;
	$('.img-responsive').eq(3).attr('src', src);
	$('#R4').html(width4+' x '+height4);
}
function setResolution5(){
	var size = $(this).val();
	width5  = Math.round(160 + 480 * size / 100).toString();
	height5 = Math.round(120 + 360 * size / 100).toString();
	var src = 'http://' + ip + ':8080/stream?topic=/' + cam5 + '/image_raw&type=mjpeg&width=' + width5 +'&height=' + height5 + '&quality=' + quality5;
	$('.img-responsive').eq(4).attr('src', src);
	$('#R5').html(width5+' x '+height5);
}

function setLed1(){
	var led = $(this).val();

	led1Publish.publish(
		new ROSLIB.Message({
			data : Math.round( $(this).val()*2.5)
		}) 
	);

	$('#L1').html(led+"%");

}

function setLed2(){
	var led = $(this).val();

	led2Publish.publish(
		new ROSLIB.Message({
			data : Math.round( $(this).val()*2.5)
		}) 
	);

	$('#L2').html(led+"%");

}

function setTilt(){
	var tilt = $(this).val();

	tiltPublish.publish(
		new ROSLIB.Message({
			data : Math.round( $(this).val())
		})
	);

	$('#TILT').html(tilt+"°");

}

function setPan(){
	var pan = $(this).val();

	panPublish.publish(
		new ROSLIB.Message({
			data : Math.round( $(this).val())
		})
	);

	$('#PAN').html(pan+"°");
}


function setPosTable(){
	var tabla;
	tabla = "<table><tr><td bgcolor='"+flipper1_color+"'>flipLF: "+flipper1_lec;
	tabla+=      "°</td><td bgcolor='"+flipper2_color+"'>flipLB: "+flipper2_lec;
	tabla+=      "°</td><td bgcolor='"+flipper3_color+"'>flipRF: "+flipper3_lec;
	tabla+=      "°</td><td bgcolor='"+flipper4_color+"'>flipRB: "+flipper4_lec;
	tabla+=      "°</td><td bgcolor='"+base_color+"'>base: "+base_lec;
	tabla+=      "°</td><td bgcolor='"+shoulder_color+"'>shoulder: "+shoulder_lec;
	tabla+=      "°</td><td bgcolor='"+elbow_color+"'>elbow: "+elbow_lec;
	tabla+=      "°</td><td bgcolor='"+roll_color+"'>roll: "+roll_lec;
	tabla+=      "°</td><td bgcolor='"+pitch_color+"'>pitch: "+pitch_lec+"°</td></tr></table>";

	$('#posTable').html( tabla);

}

function setStatusTable(){
	
	var tabla;
	tabla = "<table><tr>";
	tabla+= "<td bgcolor='white'><img src='img/logo-8.png'></td>";
	tabla+= "<td bgcolor='" + nodeStatus1_color + "'>Node1</td>";
	tabla+= "<td bgcolor='" + nodeStatus2_color + "'>Node2</td>";
	tabla+= "<td bgcolor='" + nodeStatus3_color + "'>Node3</td>";
	tabla+= "<td bgcolor='" + nodeStatus4_color + "'>Node4</td>";
	tabla+= "<td bgcolor='" + nodeStatus5_color + "'>Node5</td>";
	tabla+= "<td bgcolor='" + nodeStatus6_color + "'>Node6</td>";
	tabla+= "<td bgcolor='" + nodeStatus7_color + "'>Node7</td>";
	tabla+= "<td bgcolor='" + nodeStatus8_color + "'>Node8</td>";
	tabla+= "<td bgcolor='" + nodeStatus9_color + "'>Node9</td>";
	tabla+= "<td bgcolor='" + nodeStatus10_color + "'>Node10</td>";
	tabla+= "<td bgcolor='" + nodeStatus11_color + "'>Node11</td>";
	tabla+= "<td bgcolor='" + nodeStatus12_color + "'>Node12</td>";
	tabla+= "<td bgcolor='" + nodeStatus13_color + "'>Node13</td>";
	tabla+= "<td bgcolor='" + nodeStatus14_color + "'>Node14</td>";
	tabla+= "<td bgcolor='" + nodeStatus15_color + "'>Node15</td></tr></table>";

	$('#statusTable').html( tabla);

}

