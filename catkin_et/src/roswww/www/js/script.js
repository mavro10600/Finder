$(document).ready(principal);
var ip = '192.168.100.50';
//var ip = 'localhost';
var cam1 = 'usb_cam1', quality1 = '20', width1 = '640', height1 = '480';
var cam2 = 'usb_cam2', quality2 = '20', width2 = '640', height2 = '480';
var cam3 = 'usb_cam3', quality3 = '20', width3 = '640', height3 = '480';
var cam4 = 'usb_cam4', quality4 = '20', width4 = '640', height4 = '480';
var cam5 = 'usb_cam5', quality5 = '20', width5 = '640', height5 = '480';

var DRUM_TEXTURE = "https://keithclark.co.uk/labs/css-fps/drum2.png";
var led1Publish;
var led2Publish;

function principal(){
	//-----------ROS Connection
	// Connect to ROS.
	ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'
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
	$('input').eq(12).on('change',setQuality5);
	$('input').eq(13).on('change',setResolution5);

	// Set cams
	$('select').eq(0).on('click', setCam1);
	$('select').eq(1).on('click', setCam2);
	$('select').eq(2).on('click', setCam3);
	$('select').eq(3).on('click', setCam4);
	$('select').eq(4).on('click', setCam5);

	//pub
  	led1Publish = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/set/led1',
		messageType : 'std_msgs/Int32'
	});

	led2Publish = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/set/led2',
		messageType : 'std_msgs/Int32'
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

	//Subscribe battery
	var batteryListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/robotBattery',
		messageType : 'std_msgs/Float32'
	});

	var co2Listener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/sensor/co2',
		messageType : 'std_msgs/Int32'
	});

	var laptopBatteryListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/laptopBattery',
		messageType : 'std_msgs/Int32'
	});

	var termalListener = new ROSLIB.Topic({
		ros : ros,
		name : '/hardware/robot_state/termalData',
		messageType : 'std_msgs/Int16MultiArray'
	});

	var qrListener = new ROSLIB.Topic({
		ros : ros,
		name : '/markers',
		messageType : 'zbar_detector/Marker'
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
		var percent = Math.round(100*(levelRobotBaterry-9.9)/2.7);
		$('.progress-bar').eq(0).css("width", percent + "%");
		$('.progress-bar').eq(0).html('<strong>' + percent + '%' +'</strong>');
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
		var levelCo2 = message.data;
		console.log("CO2:"+levelCo2);
		if($('.progress-bar').eq(1).hasClass('progress-bar-success')){
		  $('.progress-bar').eq(1).toggleClass('progress-bar-success');
		}
		if($('.progress-bar').eq(1).hasClass('progress-bar-danger')){
		  $('.progress-bar').eq(1).toggleClass('progress-bar-danger');
		}
		var percent = Math.round(100*(levelCo2/100));
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

	laptopBatteryListener.subscribe(function(message){
		var levelLaptopBaterry = message.data;

		if($('.progress-bar').eq(2).hasClass('progress-bar-success')){
		  $('.progress-bar').eq(2).toggleClass('progress-bar-success');
		}
		if($('.progress-bar').eq(2).hasClass('progress-bar-danger')){
		  $('.progress-bar').eq(2).toggleClass('progress-bar-danger');
		}
		var percent = Math.round(100*(levelLaptopBaterry/100));
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

	termalListener.subscribe(function(message){
		var termalData = message.data;
		var cadena = "<table ><tr>";
		var colores =["#040530","#0e127c","#1a41bf","#1a72bf","#22bee5","#e9ed28","#edc528","#ed8a28","#c6441f","#c61f1f"];
		var temp = 0;
		for (i=0;i<termalData.length;i++){
			temp= Math.round(termalData[i]/29);
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

		var cadena = "<table ><tr><td bgcolor=\"#FFFFFF\">QR data:<br>"+qrData+"</td></tr></table>";


		$('#textQR').html(cadena);
	});


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
			data : Math.round( $(this).val())
		}) 
	);

	$('#L1').html(led+"%");

}

function setLed2(){
	var led = $(this).val();

	led2Publish.publish(
		new ROSLIB.Message({
			data : Math.round( $(this).val())
		}) 
	);

	$('#L2').html(led+"%");

}