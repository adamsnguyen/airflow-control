
<template>

    <div id="app">
        <nav id="nav" class="navbar sticky-top">
            <h1 id="title">VFD / Airflow Control with Computer Vision</h1>
            <h2 id="subtitle">A Demonstration of a machine-learning based Sensor with a VFD and PID Controls System</h2>
        </nav>
        <!-- Heading end -->
        <!-- Heading start -->

        <router-view />
        <!-- Footer -->
        <nav id="footer" class="navbar fixed-bottom navbar-light bg-light flex-shrink-0">
            <p id="footer-text">Made by Adam Nguyen</p>
        </nav>
        <!-- End of footer -->
    </div>
</template>

<script>
export default {
  name: 'App'
}

</script>

<style>
body {
    font-size: 18px;
    min-height: 100%;
    flex: 1 0 auto;
    background-color: lightgray ;
}

#app {
    font-family: 'Avenir', Helvetica, Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    text-align: center;
    color: #2c3e50;
    /* padding: 10px; */
    min-height: 100%;

}

#nav {
    width: 100%;
    display: inline-block;
    text-align: center;
    background-color: lightcyan;
    border-bottom: 2px solid black;
    border-radius: 0px;
    color: black;
    height:110px;
    top: 0;
    padding-top:15px;
}

#title {
    width: 100%;
    margin: 10px;
    font-family: 'Courier New', Courier, monospace;
}

#subtitle {
    width: 100%;
    margin: 10px;
    font-family: 'Courier New', Courier, monospace;
    font-size: 14px;
}

.fixed-bottom {
    position: fixed-bottom;
}

#footer {
    position: fixed;
    display: inline-block;
    bottom: 0;
    width: 100%;
    background-color: lightcyan;
    margin:0px;
    flex-shrink: 1;
    height: 45px;
    text-align: center;
    border-top: 2px solid black;
    border-radius: 0px;

}

#footer-text {
    display: inline-block;
    font-size: 24px;
    text-align: center;

}
</style>
