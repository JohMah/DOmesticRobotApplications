const char MAIN_page[] PROGMEM = R"=====(

<!DOCTYPE html>
<html>
<head>
<meta http-equiv="content-type" content="text/html; charset=ISO-8859-1">
    <title>ESP8266 Robot_1</title>
  

<meta name="viewport" content="width=device-width, initial-scale=1">

<style type="text/css">
body {
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif, "Apple Color Emoji", "Segoe UI Emoji", "Segoe UI Symbol";
}

.item, .grid-item {
  background-color: #123456;
  color: #ff;
  padding: 10px;
  border-radius: 50%;
  border: 2px;
}
.item > a, .grid-item > a {
  display: block;
}
.item > a:link, .item > a:visited, .grid-item > a:link, .grid-item > a:visited {
  text-decoration: none;
  outline: none;
  color: #FFF;
}
.item > a:hover, .item > a:focus, .grid-item > a:hover, .grid-item > a:focus {
  background-color: #17242D;
}

h2 {
  margin: 0;
  margin-left: auto;
  margin-right: auto;
}

arrow {
  font-family:Arial, sans-serif;
  font-size:200%;
  font-weight: bold;
  text-align:center;
  margin-left: auto;
  margin-right: auto;
}

.grid {
  margin: 0 0 40px;
  list-style: none;
  padding: 0;
}

.dark,.item:focus  {
  background-color: #17242D;
  outline: none;
}

.light{
  background-color: #87B3D6;
  color: #17242D;
}

.grid {
  display: grid;
  grid-template-columns: 50px 50px 50px 50px;
  //grid-template-rows: 100px 100px 100px 100px;
  grid-auto-rows: 50px;
  grid-gap: 20px;
  transition: all 1s;
}

.buttonr, .buttong, .buttonb {
    display: inline-flex; /* keep the inline nature of buttons */
    align-items: center; /* this is default */
    border-radius: 20%;
    text-align:center;
    border: 5px solid #555555;
    box-shadow: 2px 9px #999;
}
.buttonr{
    background-color: #f44336; /* Red */
}

.buttonr:active {
  background-color: #7A211b;
  box-shadow: 0 5px #666;
  transform: translateY(4px);
}
.buttong{
    background-color: #4CAF50; /* Green */
}
.buttong:active {
  background-color: #265728;
  box-shadow: 0 5px #666;
  transform: translateY(4px);
}
.buttonb{
    background-color: #4C50f4; /* Blue */
}
.buttonb:active {
  background-color: #26287a;
  box-shadow: 0 5px #666;
  transform: translateY(4px);
}

dt {
  font-weight: bold;
  margin-bottom: 0.5rem;
}

.label1, .label2, .label {
    display: inline-block;
    width: 80px;
}

.label {
    width: 50px;

}

.label1 {
  background-color: #ffb0a0; /* Red */
}
.label2 {
    background-color: #A0ffA0; /* Green */
}


</style>


<div class="grid js-grid">
  <article class="buttonr" type="button" onclick="RControl('0')">
    <arrow> &#8624 </arrow>
  </article>
  <article class="buttonr" type="button" onclick="RControl('1')">
    <arrow> &#8598 </arrow>
  </article>
  <article class="buttonr" type="button" onclick="RControl('2')">
    <arrow> &#8599 </arrow>
  </article>
  <article class="buttonr" type="button" onclick="RControl('3')">
    <arrow> &#8625 </arrow>
  </article>
  <article class="buttonb" type="button" onclick="RControl('4')">
    <h2>4</h2>
  </article>
  <article class="buttonb" type="button" onclick="RControl('5')">
    <h2>5</h2>
  </article>  
  <article class="buttonb" type="button" onclick="RControl('6')">
    <h2>6</h2>
  </article>
  <article class="buttonb" type="button" onclick="RControl('7')">
    <h2>7</h2>
  </article>
  <article class="buttonb" type="button" onclick="RControl('8')">
    <h2>8</h2>
  </article>
  <article class="buttonb" type="button" onclick="RControl('9')">
    <h2>9</h2>
  </article>  
  <article class="buttonb" type="button" onclick="RControl('A')">
    <h2>A</h2>
  </article>
  <article class="buttonb" type="button" onclick="RControl('B')">
    <h2>B</h2>
  </article>
  <article class="buttong" type="button" onclick="RControl('C')">
    <arrow> &#8626 </arrow>
  </article>  
  <article class="buttong" type="button" onclick="RControl('D')">
    <arrow> &#8601 </arrow>
  </article>
  <article class="buttong" type="button" onclick="RControl('E')">
    <arrow> &#8600 </arrow>
  </article>
  <article class="buttong" type="button" onclick="RControl('F')">
    <arrow> &#8627 </arrow>
  </article>
</div>
<div>
   <label>P</label>
   <label class="label" id="P"> &nbsp </label>
   <label>I</label>
   <label class="label" id="I"  value=""> &nbsp </label>
   <label>D</label>
   <label class="label" id="D"  value=""> &nbsp </label>
</div> <br>
<div>
   <label>Left</label>
   <label class="label1" id="PosL"> &nbsp </label>
   <label>Right</label>
   <label class="label2" id="PosR"> &nbsp </label><br>
</div><br>


<script>
var connection = new WebSocket('ws://192.168.2.60/ws');
connection.onopen = function () {
};
connection.onerror = function (error) {
  console.log('WebSocket Error ', error);
};
connection.onmessage = function (e) {
  console.log('Server: ', e.data);
};
connection.onclose = function () {
  console.log('WebSocket connection closed');
};

function RControl(number) {
  connection.send(number);
}
</script>

<script>
document.querySelector('.js-button').addEventListener('click', function() {
  console.log('test')
  document.querySelector('.js-grid').classList.toggle('grid--full')
})

</script>
<script>
setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getLoadData();
}, 2000); //2000mSeconds update rate

function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ADCValue").innerHTML = this.responseText;
      
    }
  };
  xhttp.open("GET", "getB", true);
  xhttp.send();
}

function getLoadData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var obj = JSON.parse(this.responseText);
      document.getElementById("P").innerHTML = obj.P;
      document.getElementById("I").innerHTML = obj.I;
      document.getElementById("D").innerHTML = obj.D;
      document.getElementById("PosL").innerHTML = obj.PosL;      
      document.getElementById("PosR").innerHTML = obj.PosR;
    }
  };
  
  xhttp.open("GET", "getLoad", true);
  xhttp.send();
}
</script>
</body>
</html>

)=====";
