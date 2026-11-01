var IProbot = "10.0.0.111";

// callbacks
document.getElementById("btn_off").addEventListener("click",function (evt){
    disable(IProbot);
});
document.getElementById("btn_led_off").addEventListener("click",function (evt){
    setLeds(IProbot,["off","off","off","off","off","off"]);
});
document.getElementById("btn_led_red").addEventListener("click",function (evt){
    setLeds(IProbot,["red","red","red","red","red","red"]);
});
document.getElementById("btn_led_sequence").addEventListener("click",function (evt){
    setLeds(IProbot,["blue","off","off","off","off","off"]);
    sleep(0.5);
    setLeds(IProbot,["green","blue","off","off","off","off"]);
    sleep(0.5);
    setLeds(IProbot,["red","green","blue","off","off","off"]);
    sleep(0.5);
    setLeds(IProbot,["yellow","red","green","blue","off","off"]);
    sleep(0.5);
    setLeds(IProbot,["cyan","yellow","red","green","blue","off"]);
    sleep(0.5);

    setLeds(IProbot,["purple","cyan","yellow","red","green","blue"]);
    sleep(0.5);
    setLeds(IProbot,["white","cyan","yellow","red","green","blue"]);
    sleep(0.5);
    setLeds(IProbot,["white","white","yellow","red","green","blue"]);
    sleep(0.5);
    setLeds(IProbot,["white","white","white","red","green","blue"]);
    sleep(0.5);
    setLeds(IProbot,["white","white","white","white","green","blue"]);
    sleep(0.5);
    setLeds(IProbot,["white","white","white","white","white","blue"]);
    sleep(0.5);
    setLeds(IProbot,["white","white","white","white","white","white"]);
    sleep(0.5);
});
document.getElementById("btn_zero").addEventListener("click",function (evt){
    setMotors(IProbot,[[0,0,0,0,0,0,2]]);
});
document.getElementById("btn_read").addEventListener("click",function (evt){
    getAngles(IProbot);
});
document.getElementById("btn_prep").addEventListener("click",function (evt){
  setMotors(IProbot,[[0,-80,0,0,80,25, 3]]);
});
document.getElementById("btn_dance").addEventListener("click",function (evt){
    let sequence = [
      // 6 positions + durée en secondes

      [0,-80,0,0,90,25, 1], // remue
      [0,-80,0,0,80,25, 1], // revient
      [0,-80,0,0,80,25, 2], // pause

      [0,-80,0,0,90,25, 1], // remue
      [0,-80,0,0,80,25, 1], // revient
      [0,-80,0,0,80,25, 5], // pause

      [0,-70,40,0,-20,25, 5], // leve
      [0,-70,45,0,-25,25, 1], // effort
      [0,-70,43,0,-23,25, 1], // effort
      [0,-80,0,0,80,25, .5], // retombe
      [0,-80,0,0,80,25, 1.5], //pause


      [0,-80,0,0,90,25, 1], // remue
      [0,-80,0,0,80,25, 1], // revient
      [0,-80,0,0,80,25, 2], // pause

      [-20,-80,0,80,80,25, 2], // prétourne
      [45,-80,0,120,40,25, 1.8], // tourne
      [90,0,0,150,-40,-50, 2], // redresse
      [90,0,0,150,00,20, 3.5], // baille
    ]
    setMotors(IProbot,sequence);
});



// off, red, green, blue, cyan, yellow, purple, white
setLeds = function(IP,listLED){
    valstr = JSON.stringify(listLED);
    fetch("http://"+IProbot+":5000/set_leds",{
        method: 'POST',
        body: valstr,
        headers: {"Content-Type":"application/json;charset=UTF-8"}
    })
    .catch(err => console.log(err));
}

disable = function(IP){
    setMotors(IProbot,[[NaN,NaN,NaN,NaN,NaN,NaN,1]]);
}

setMotors = function(IP,listPos){
    valstr = JSON.stringify(listPos);
    fetch("http://"+IProbot+":5000/set_positions",{
        method: 'POST',
        body: valstr,
        headers: {"Content-Type":"application/json;charset=UTF-8"}
    })
    .then(res => res.json())
    .then(data => {
        document.getElementById("positions").innerHTML = data.result;
    })
    .catch(err => console.log(err));
}

getAngles = function(IP){
    var mesPos={};
    fetch("http://"+IProbot+":5000/get_positions",{
        method: 'GET',
        headers: {"Content-Type":"application/json;charset=UTF-8"}
    })
    .then(res => res.json())
    .then(data => {
        mesPos.m1 = Math.round(data[0]);
        mesPos.m2 = Math.round(data[1]);
        mesPos.m3 = Math.round(data[2]);
        mesPos.m4 = Math.round(data[3]);
        mesPos.m5 = Math.round(data[4]);
        mesPos.m6 = Math.round(data[5]);
        document.getElementById("positions").innerHTML = "["+mesPos.m1+","+mesPos.m2+","+mesPos.m3+","+mesPos.m4+","+mesPos.m5+","+mesPos.m6+"]";
    })
    .catch(err => console.log(err));
}

function sleep(delaySec) {
    var start = new Date().getTime();
    while (new Date().getTime() < start + delaySec*1000);
}
