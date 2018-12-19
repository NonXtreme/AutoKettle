const APPID = "EmbeddedLab2018";
const KEY = "IJ40DCVoUQAXW23";
const SECRET = "tyyTIMnies37G3nTIfjCBjP45";

const WEBALIAS = "HTML_web";
const thing1 = "esp8266";

function init() {
  $("#controlpanel").hide();

  var microgear = Microgear.create({
    key: KEY,
    secret: SECRET,
    alias : WEBALIAS
  });

  microgear.on('message', function(topic,msg) {
    /* ADD: parse message from nodemcu and send them to appropriate functions
    */
  });
  
  microgear.on('connected', function() {
    microgear.setAlias(WEBALIAS);
    $("#connectmsg").hide();
    $("#controlpanel").show();
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

  $("#timepicker").wickedpicker({
    twentyFour: true,
    title: 'Select Time',
    timeSeparator: ':'
  });
}

window.onload = init;
