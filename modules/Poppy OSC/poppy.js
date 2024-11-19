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
	local.send("/handshake", [util.getIPs()[0], local.parameters.oscInput.localPort.get(), "chataigne"]);
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

function goToZero()
{
	local.send("/motors/set", [0]);
}

function setAngle(duration, index, pos)
{
	local.send("/motors/set", [duration, index, pos]);
}


function addToAngle(duration, index, val)
{
	local.send("/motors/add", [index, val]);
}