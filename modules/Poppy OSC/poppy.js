IPS = ["10.0.0.101", "10.0.0.110", "10.0.0.103", "10.0.0.111", "10.0.0.112"];

function init()
{
	handshake();
}

function moduleParameterChanged(param)
{    
	script.log("Param changed : "+param.name);
	
	if (param.name == "model")
		local.parameters.oscOutputs.oscOutput.remoteHost.set(IPS[local.parameters.model.get()]);

	if (param.name == "localPort" || param.name == "local" || param.name == "remoteHost" || param.name == "remotePort")
		handshake();
}

function handshake()
{
	local.send("/handshake", [util.getIPs()[0], local.parameters.oscInput.localPort.get(), "chataigne-"+util.getOSInfos().computerName]);
} 

function moduleValueChanged(value) 
{ 
	if(value.isParameter()) { script.log("Module value changed : "+value.name+" > "+value.get()); }
	else 
	{	
		script.log("Module value triggered : "+value.name);
	}
}

function oscEvent(address, args) {
	script.log(address);
	script.log(args);
	if (address == "/angle/1") local.values.angle1.set(args[0]);
	if (address == "/angle/2") local.values.angle2.set(args[0]);
	if (address == "/angle/3") local.values.angle3.set(args[0]);
	if (address == "/angle/4") local.values.angle4.set(args[0]);
	if (address == "/angle/5") local.values.angle5.set(args[0]);
	if (address == "/angle/6") local.values.angle6.set(args[0]);
}

function setLed(index, color)
{
	local.send("/leds/set", [index, color]);
}

function setLeds(led1, led2, led3, led4, led5, led6)
{
	local.send("/leds/set", [led1, led2, led3, led4, led5, led6]);
}

function setAllLeds(color)
{
	setLeds(color, color, color, color, color, color);
}

function getAngles(servo, value)
{
	handshake();
	local.send("/motors/get");
}

function disable(index)
{
	local.send("/motors/set", [index, None]);
}

function disableAll()
{
	local.send("/motors/set", [None]);
}

function goToZero(duration)
{
	local.send("/motors/set", [duration, 0]);
}

function gotToAngle(duration, index, pos)
{
	local.send("/motors/set", [duration, index, pos]);
}

function goToPose(duration, angle1, angle2, angle3, angle4, angle5, angle6)
{
	local.send("/motors/set", [duration, angle1, angle2, angle3, angle4, angle5, angle6]);
}

function addToAngle(duration, index, val)
{
	local.send("/motors/add", [duration, index, val]);
}