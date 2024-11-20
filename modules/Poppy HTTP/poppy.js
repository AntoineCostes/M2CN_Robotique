/*
function moduleParameterChanged(param){    script.log("Param changed : "+param.name); }

function moduleValueChanged(value) { if(value.isParameter()) { script.log("Module value changed : "+value.name+" > "+value.get());
}else {
script.log("Module value triggered : "+value.name);
}
}*/

function dataEvent(data, requestURL) {
	//script.log("Data received, request URL :"+requestURL+"\nContent :\n" +data);

	// get angles
	if (requestURL == local.parameters.baseAddress.get()+"get_positions")
	{
		local.values.angle1.set(data[0]);
		local.values.angle2.set(data[1]);
		local.values.angle3.set(data[2]);
		local.values.angle4.set(data[3]);
		local.values.angle5.set(data[4]);
		local.values.angle6.set(data[5]);
	}
}

var colors = ["off", "red", "green", "yellow", "blue", "purple", "cyan", "white"];
function setLeds(led1, led2, led3, led4, led5, led6)
{
	script.log(led1);
	var params = {};
	params.dataType = "json";
	params.extraHeaders = "Content-Type: application/json;charset=UTF-8";
	var payload = [
		colors[led1],
		colors[led2],
		colors[led3],
		colors[led4],
		colors[led5],
		colors[led6]
	];
	params.payload = payload;

	local.sendPOST("set_leds", params);
}

function setAllLeds(color)
{
	setLeds(color, color, color, color, color, color);
}

function getAngles(servo, value)
{
	local.sendGET("get_positions", "json");
}

function disable()
{
	script.log("Disable");

	goTo(1, null, null, null, null, null, null);
/*
	var params = {};
	params.dataType = "json";
	params.extraHeaders = "Content-Type: application/json;charset=UTF-8";
	var payload = [[NaN, NaN, NaN, NaN, NaN, NaN, 1]];
	params.payload = payload;

	local.sendPOST("set_positions", params);*/
}

function goTo(duration, pos1, pos2, pos3, pos4, pos5, pos6)
{
	var params = {};
	params.dataType = "json";
	params.extraHeaders = "Content-Type: application/json;charset=UTF-8";
	var payload = [[pos1, pos2, pos3, pos4, pos5, pos6, duration]];
	params.payload = payload;

	local.sendPOST("set_positions", params);
}

function enableOSC()
{
	var params = {};
	params.dataType = "json";
	params.extraHeaders = "Content-Type: application/json;charset=UTF-8";

	local.sendPOST("enable_osc", "json");
}
