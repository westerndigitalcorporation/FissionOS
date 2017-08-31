// General functions

function get_url(url)
{
	var request = new XMLHttpRequest();

	request.open("GET", url, false);
	request.send(null);

	if (request.status != 200)
	{
		return null;
	}

	return request.responseText;
}

function update_id_from_url(id, url)
{
        document.getElementById(id).innerHTML = get_url(url);
}

function update_id(id, contents)
{
	document.getElementById(id).innerHTML = contents;
}

function update_value(id, contents)
{
	document.getElementById(id).value = contents;
}

function get_xml(url)
{
	var request = new XMLHttpRequest();

	request.open("GET", url, false);
	request.send(null);

	if (request.status != 200)
	{
		return;
	}

	return request.responseXML;
}

function get_value_from_xml(xml, id)
{
	var value;

	try
	{
		value = xml.getElementsByTagName(id)[0].childNodes[0].nodeValue;
	}
	catch(err)
	{
		return "";
	}

	return value;
}

var progress_timer;
var current_progress = 100;
var shown_progress = 100;
function display_progress()
{
	var progress_bar = document.getElementById("progress");

	if (shown_progress < current_progress)
	{
		shown_progress++;
	}

	progress_bar.style.width = shown_progress + "%";

	if (shown_progress != 100)
	{
		progress_timer = window.setTimeout(display_progress, 25);
	}
}

function complete_progress(evt)
{
	var progress_bar = document.getElementById("progress");

	current_progress = 100;

	if (evt.target.status != 200)
	{
		return;
	}

	update_id(evt.target.status_id, evt.target.responseText);
}

function update_progress(evt)
{
	var percentage = (evt.loaded * 100) / evt.total;

	current_progress = percentage;
}

function post_upload(url, boundary, file, status_id)
{
	var request = new XMLHttpRequest();
	var formdata = new FormData();

	formdata.append(boundary, file);

	request.status_id = status_id;
	request.upload.addEventListener("progress", update_progress, false);
	request.addEventListener("load", complete_progress, false);
	request.addEventListener("error", complete_progress, false);
	request.addEventListener("abort", complete_progress, false);
	request.open("POST", url);
	request.setRequestHeader("Content-type", "multipart/form-data;");
	request.send(formdata);

	shown_progress = 0;
	current_progress = 0;
	display_progress();
}

//
// Net page specific functions
//
function show_static_ip(show)
{
	var static_ip_config = document.getElementById("static_ip_config");

	static_ip_config.style.visibility = "hidden";
	if (show)
	{
		static_ip_config.style.visibility = "visible";
	}
}

function show_hostname(show)
{
	var hostname_config = document.getElementById("hostname_config");

	hostname_config.style.visibility = "hidden";
	if (show)
	{
		hostname_config.style.visibility = "visible";
	}
}

function save_net_config()
{
	var url = "net_config.cgi?";
	var result;

	url = url + "macaddr=" + document.getElementById("macaddr").value;
	url = url + "&dhcp=" + document.getElementById("dhcpon").checked;
	url = url + "&hostname=" + document.getElementById("hostname").value;
	url = url + "&ipaddr=" + document.getElementById("ipaddr").value;
	url = url + "&netmask=" + document.getElementById("netmask").value;
	url = url + "&gateway=" + document.getElementById("gateway").value;

	result = get_url(url);
	if (result == null)
	{
		return "ERROR: Save Failed";
	}

	return result;
}

function update_net_fields()
{
	var data = get_xml("net_xml.cgi");

	if (get_value_from_xml(data, "dhcp") == "true")
	{
		show_hostname(1);
		show_static_ip(0);
		document.getElementById("dhcpon").checked = "checked";
	}
	else
	{
		show_hostname(0);
		show_static_ip(1);
		document.getElementById("dhcpoff").checked = "checked";
	}

	update_value("hostname", get_value_from_xml(data, "hostname"));
	update_value("macaddr", get_value_from_xml(data, "macaddr"));
	update_value("ipaddr", get_value_from_xml(data, "ipaddr"));
	update_value("netmask", get_value_from_xml(data, "netmask"));
	update_value("gateway", get_value_from_xml(data, "gateway"));
}

function update_net_contents()
{
	update_id_from_url("main_contents", "net.html");
	update_id("title_contents", "Network Configuration");
	update_net_fields();
	update_id_from_url("net_config", "net_config.cgi");
}

function submit_net_config()
{
	update_id("save_status", save_net_config());
}

//
// Target Page Functions
//
function update_target_contents()
{
	update_id_from_url("main_contents", "target.html");
	update_id("title_contents", "Target Status");

	update_target_fields();
	update_id_from_url("target_config", "target.cgi");
}

function update_target_config()
{
	var url = "target.cgi?";
	var result;

	result = get_url(url);
	if (result == null)
	{
		return 0;
	}

	update_id("target_config", result);
	update_target_fields();

	return 0;
}

function update_target_fields()
{
	var data = get_xml("target_xml.cgi");
}

function submit_target_state(state)
{
	var url = "target.cgi?state=" + state;
	var result;

	result = get_url(url);
	if (result == null)
	{
		return 0;
	}

	update_id("target_config", result);
	update_target_fields();

	return 0;
}

function submit_target_firmware()
{
	var file = document.getElementById("target_firmware").files[0];

	submit_target_state('resume');
	update_id("save_status", "Uploading...");
	post_upload("target_firmware.cgi", "target_firmware", file, "save_status");
}


//
// Util Page Functions
//
function update_util_contents()
{
	update_id_from_url("main_contents", "util.html");
	update_id("title_contents", "Utilities");
	update_id_from_url("firmware_version", "firmware_version.cgi");
}

function reload_homepage()
{
	window.location = "/";
}

function submit_bootloader()
{
	update_id_from_url("save_status", "bootloader.cgi");

	window.setTimeout(reload_homepage, 10000);
}

function submit_reset()
{
	update_id_from_url("save_status", "reset.cgi");

	window.setTimeout(reload_homepage, 10000);
}
