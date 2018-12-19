const APPID = "EmbeddedLab2018";
const KEY = "IJ40DCVoUQAXW23";
const SECRET = "tyyTIMnies37G3nTIfjCBjP45";

const WEBALIAS = "HTML_web";
const thing1 = "esp8266";

function init() {
  var microgear = Microgear.create({
    key: KEY,
    secret: SECRET,
    alias : WEBALIAS
  });

  microgear.on('message',function(topic,msg) {
    document.getElementById("raw_data").innerHTML = "Data from Node MCU = " + msg;
    document.getElementById("get_topic").innerHTML = "Topic = " + topic;
    var split_msg = msg.split("/"); //String data = "/" +String(Humidity) + "/" + String(Temp);
    console.log(msg);  // for debug
    if(typeof(split_msg[0])!='undefined' && split_msg[0]==""){
      document.getElementById("Humidity_temp").innerHTML = "Humidity = " + split_msg[1] + " % ,Temp = " + split_msg[2] + " c";
    }
  });
  
  microgear.on('connected', function() {
    microgear.setAlias(WEBALIAS);
    $("#connected_NETPIE").text("Connected to NETPIE");
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

  var tstkub = setInterval(runSi,1000);
  $(".timepicker").wickedpicker({
    twentyFour: true,
    title: 'Select Time',
    timeSeparator: ':'
  });
}

function runSi() {
  a = new Date();
  console.log(new Date().toLocaleTimeString('en-US', { hour12: false,hour: "numeric", minute: "numeric"}));
}

window.onload = init;
