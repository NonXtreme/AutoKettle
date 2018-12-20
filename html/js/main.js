const APPID = "EmbeddedLab2018";
const KEY = "IJ40DCVoUQAXW23";
const SECRET = "tyyTIMnies37G3nTIfjCBjP45";

const WEBALIAS = "AutoKettle";
const thing1 = "esp8266";

var tst;
var microgear;
function init() {
  $("#controlpanel").hide();

  microgear = Microgear.create({
    key: KEY,
    secret: SECRET,
    alias : WEBALIAS
  });

  microgear.on('message', function(topic,msg) {
    /* ADD: parse message from controller and send them to appropriate functions:
    controllerInit(watper, scheon, time, duration) for all the data requested by init()
    updateWater(watper) for the live update of water data only

    watper = current water percentage in Number
    scheon = current schedule enabled state in Boolean
    time = current schedule time in HH:mm String, optional
    duraion = current on duration in Number, optional
    */
	updateWater(msg.split(' ')[1]);
  });
  
  microgear.on('connected', function() {
    microgear.setAlias(WEBALIAS);
	//---
	controllerInit(0,false,undefined,10);
	microgear.subscribe("/weight");
	updateTime()
  });
  
  microgear.on('present', function(event) {
    console.log(event);
  });
  
  microgear.on('absent', function(event) {
    console.log(event);
  });

  microgear.resettoken(function(err) {
    microgear.connect(APPID);
  });

  // ADD: request the controller to send the data for controllerInit()
}

function controllerInit(watper, scheon, time, duration) {
  var tpconf = {
    twentyFour: true,
    title: 'Select Time',
    timeSeparator: ':',
    afterShow: sendScheduleTime
  };
  if (time != undefined) tpconf.now = time;
  $("#connectmsg").hide();
  $("#controlpanel").show();
  $("#timepicker").wickedpicker(tpconf);
  $("#duration").val(duration || 10);
  enableScheduleFields(scheon);
  updateWater(watper);

  tst = setInterval(updateTime,30000)
  
  $("#btnoff").click(function() {
    // ADD: tell the controller to turn off the kettle
	microgear.publish("/send","R0       ")
	console.log("R0       ");	
  });

  $("#btnon").click(function() {
    // ADD: tell the controller to turn on the kettle
	microgear.publish("/send","R1       ")
	console.log("R1       ");
  });

  $("#chkauto").click(function() {
    var checked = this.checked;
    enableScheduleFields(checked);
    // ADD: tell the controller to update the schedule on/off setting
	if(!checked) {
		console.log("AF       ");
		microgear.publish("/send","AF       ");
	} else {
		sendScheduleTime()
	}
  });

  $("#duration").change(function() {
    var dur = Number.parseInt($(this).val());
    if (Number.isNaN(dur)) return;
    // ADD: tell the controller to update the duration setting
  });
  
}

function updateWater(watper) {
  var txt = Math.min(Math.max(watper, 0), 100).toFixed(1) + "%";
  $("#watergfx div").height(txt);
  $("#wateramt").text(txt);
}

function enableScheduleFields(enabled) {
  $("#chkauto")[0].checked = enabled;
  $(".field input").each(function() {
    this.disabled = !enabled;
  });
}

function sendScheduleTime() {
  var time = $("#timepicker").wickedpicker('time');
  // ADD: tell the controller to update the time setting
  console.log("A"+time+"   ");
  microgear.publish("/send","A"+time+"   ") 
}
function updateTime() {
	a = new Date()
	console.log("T"+a.toTimeString().split(' ')[0])
	microgear.publish("/send","T"+a.toTimeString().split(' ')[0]) 
}
  
window.onload = init;
