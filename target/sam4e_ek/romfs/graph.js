
var canvases = [];
var canvas_count = 0;
var border_offset = 5;

//
// Data Handling Function
//

//
// Initialize the data ringbuffer with zeros, set the position to 0
//
function data_init(canvas, url)
{
	var i;

	canvas.dataset     = [];
	canvas.datacurrent = 0;
	canvas.datasize    = 200;
	canvas.maxvalue    = 0;
	canvas.gridlinesx  = 10;
	canvas.gridlinesy  = 3;
	canvas.xoffset     = 30;
	canvas.yoffset     = 20;
	canvas.linecolor   = 'rgba(200, 0, 0, 1)';
	canvas.url         = url;

	for (i = 0; i < canvas.datasize; i++)
	{
		canvas.dataset[i] = 0;
	}

	datacurrent = 0;
}

//
// Append newdata to end of the canvas ringbuffer
//
function dataset_append(canvas, newdata)
{
	var index;

	// Set next position
	canvas.datacurrent++;
	if (canvas.datacurrent >= canvas.datasize)
	{
		canvas.datacurrent = 0;
	}

	canvas.dataset[canvas.datacurrent] = newdata;

	if (newdata > canvas.maxvalue)
	{
		canvas.maxvalue = newdata;
	}
}

//
// Request the data from the server
//
function request_data(canvas)
{
	var request = new XMLHttpRequest();
	var data;

	request.open("GET", canvas.url, false);
	request.send(null);

	if (request.status != 200)
	{
		return;
	}

	data = request.responseText * 1;
	dataset_append(canvas, data);
}


//
// Canvas Handling Functions
//
function add_canvas(canvas, url)
{
	canvases[canvas_count] = canvas;
	data_init(canvas, url);

	canvas_count++;
}

//
// Clear the canvas so we can redraw on it
//
function clear_canvas(context, canvas) {
	context.clearRect(0, 0, canvas.width, canvas.height);
	var w = canvas.width;

	canvas.width = 1;
	canvas.width = w;
}

//
// Render the background grid
//
function render_grid(context, canvas)
{
	var i;

	context.beginPath();

	context.strokeStyle = 'rgba(220, 220, 220, 1)';

	context.moveTo(canvas.xoffset, canvas.yoffset);
	context.lineTo(canvas.xoffset, canvas.height - border_offset);
	context.lineTo(canvas.width - border_offset,
		       canvas.height - border_offset);
	context.lineTo(canvas.width - border_offset,
		       canvas.yoffset);
	context.lineTo(canvas.xoffset, canvas.yoffset);

	context.stroke();
	context.closePath();

	for (i = 0; i < canvas.gridlinesx; i++)
	{
		var gridwidth = (canvas.width - border_offset - canvas.xoffset) / canvas.gridlinesx;
		var offset = (gridwidth * i) + canvas.xoffset;

		context.beginPath();
		context.moveTo(offset, canvas.yoffset);
		context.lineTo(offset, canvas.height - border_offset);
		context.stroke();
		context.closePath();
	}

	for (i = 0; i < canvas.gridlinesy; i++)
	{
		var gridheight = (canvas.height - border_offset - canvas.yoffset) / canvas.gridlinesy;
		var offset = (gridwidth * i) + canvas.yoffset;

		context.beginPath();
		context.moveTo(canvas.xoffset, offset);
		context.lineTo(canvas.width - border_offset, offset);
		context.stroke();
		context.closePath();
	}
}

//
// Perform the actual graph drawing from the ringbuffer data
//
function render_graph(context, canvas)
{
	var width   = canvas.width - canvas.xoffset - border_offset;
	var height  = canvas.height - canvas.yoffset - border_offset;
	var index   = canvas.datacurrent + 1;
	var x       = 0;
	var y;
	var i;

	render_grid(context, canvas);

	context.fillText(0, 0, height + canvas.yoffset);
	context.fillText(canvas.maxvalue, 0, 20);

	context.beginPath();

	// Line color and blur
	context.strokeStyle = canvas.linecolor;
	context.lineWidth = 2;
	context.shadowOffsetX = 2;
	context.shadowOffsetY = 2;
	context.shadowBlur = 3;
	context.shadowColor = 'rgba(0, 0, 0, 0.4)';

	for (i = 0; i < canvas.datasize; i++)
	{

		if (index >= canvas.datasize)
		{
			index = 0;
		}

		x += width / canvas.datasize;

		// Don't divide by zero
		if (canvas.maxvalue == 0)
			y = height;
		else
			y = height - ((canvas.dataset[index] * height) / canvas.maxvalue);

		if (i == 0)
		{
			context.moveTo(x + canvas.xoffset, y + canvas.yoffset);
		}
		else
		{
			context.lineTo(x + canvas.xoffset, y + canvas.yoffset);
		}

		index++;
	}

	context.stroke();
	context.closePath();
}

