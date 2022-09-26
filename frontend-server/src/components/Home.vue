<template>
  <div>
    <div class="container-fluid">
      <div class="text-center" id="div-1">
        <div>
          <div class="container-fluid-2">
            <div class="mode-col">
              <h1 id="mode-title">Mode: {{ mode }}</h1>
            </div>

            <div class=" button-col">
              <button id="start" class="button" @click="motorStart">
                Start
              </button>
            </div>

            <div class=" button-col">
              <button id="stop" class="button" @click="motorStop">Stop</button>
            </div>
          </div>

          <!-- Panel div start -->
          <div class="panel panel-primary">
            <div class="panel-heading">
              <div class="container">
                <div class="row">
                  <div class="col-xl-4">
                    <h3 class="panel-title" id="mode">
                      Status: {{ connStatus }}
                    </h3>
                  </div>
                  <div class="col-xl-8">
                    <div class="container" id="radio-button-choices">
                      <div class="row">
                        <div class="col-sm-2 axis-select-div">
                          <p class="axis-select-title">Y-Axis:</p>
                        </div>
                        <div class="col-sm-10">
                          <b-form-radio-group
                            @change="onPrimaryChange($event)"
                            id="y--axis-select"
                            v-model="primarySelected"
                            :options="options"
                            name="primary-y-options"
                          ></b-form-radio-group>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
            <div class="panel-body">
              <!-- Chart container -->
              <div id="chart_container" class="chart-padding">
                <div id="y_axis" class="chart-padding"></div>
                <div id="demo_chart" ref="panel"></div>
                <!-- <div id="y_axis_secondary" class="chart-padding"></div> -->
              </div>
              <!-- End of chart container -->
            </div>
            <div class="panel-footer">
              <!-- <p v-if="displayedValues.length > 0">
                <small>
                  <span v-bind:style="{ color: dvColors.v1 }">{{ displayedValues[0].v1 }} </span>V
                  | <span v-bind:style="{ color: dvColors.v2 }">{{ displayedValues[0].v2 }}
                  </span>V
                  | <span v-bind:style="{ color: dvColors.v3 }">{{ displayedValues[0].v3 }}
                  </span>V
                </small>
              </p> -->
            </div>
          </div>
          <!-- Panel div end -->
        </div>

        <div class="container-fluid-2">
          <div
            class="col2 indicator-box"
            id="speed"
            v-if="displayedValues['motorSpeed'].length > 0"
          >
            <h2 id="vfd-value">{{ displayedValues['motorSpeed'][0] }}</h2>
            <div class="indicator">
              <p>Motor Speed [rpm]</p>
            </div>
          </div>
          <div
            class="col2 indicator-box"
            id="air-speed"
            v-if="displayedValues['airflowSpeed'].length > 0"
          >
            <h2 id="air-speed-value">
              {{ displayedValues['airflowSpeed'][0] }}
            </h2>
            <div class="indicator">
              <p>Airflow speed [cfm]</p>
            </div>
          </div>
          <div
            class="col2 indicator-box"
            id="height"
            v-if="displayedValues['objectHeight'].length > 0"
          >
            <h2 id="height-value">{{ displayedValues['objectHeight'][0] }}</h2>
            <div class="indicator">
              <p>Object Height [mm]</p>
            </div>
          </div>
          <div
            class="col2 indicator-box"
            id="voltage"
            v-if="displayedValues['voltage'].length > 0"
          >
            <h2 id="v-f-value">{{ displayedValues['voltage'][0] }}</h2>
            <div class="indicator">
              <p>Voltage</p>
            </div>
          </div>
        </div>
      </div>

      <div id="div-2">
        <div class="container-fluid-3">
          <div id="div-3">
            <b-card no-body>
              <b-tabs card>
                <b-tab title="Manual Control" active>
                 

                  <b-card-text>
                    <h4>Manual VFD Control</h4>
                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >RPM:
                          </label>
                          <b-form-input
                            id="inline-form-setpoint"
                            class="mb-2 mr-sm-2 mb-sm-0 form-input"
                            placeholder="800"
                            v-model="vfdSetpoint"
                            @keypress="isNumber($event)"
                          ></b-form-input>

                          <b-button
                            variant="primary"
                            v-on:click="setPointRequestFunc()"
                            >Apply</b-button
                          >
                          <br />
                          <p class="description-text">
                            Set RPM between 800 and 1600
                          </p>
                        </b-form>
                      </div>

                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >Coarse control:
                          </label>
                          <b-button
                            class="button-side-margins"
                            v-model="vfdSetpoint"
                            variant="primary"
                            v-on:click="setPointCoarseIncreaseFunc()"
                            >Increase</b-button
                          >
                          <b-button
                            variant="secondary"
                            class="button-side-margins"
                            v-model="vfdSetpoint"
                            v-on:click="setPointCoarseDecreaseFunc()"
                            >Decrease</b-button
                          >
                        </b-form>
                        <br />
                        <p class="description-text"></p>
                      </div>

                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >Fine control:
                          </label>
                          <b-button
                            class="button-side-margins"
                            variant="primary"
                            v-model="vfdSetpoint"
                            v-on:click="setPointFineIncreaseFunc()"
                            >Increase</b-button
                          >
                          <b-button
                            variant="secondary"
                            class="button-side-margins"
                            v-model="vfdSetpoint.text"
                            v-on:click="setPointFineDecreaseFunc()"
                            >Decrease</b-button
                          >
                        </b-form>
                        <br />
                        <p class="description-text"></p>
                      </div>
                    </form>

                    <br />
                    <h4>Damper Motor Control</h4>
                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >Fine control:
                          </label>
                          <b-button
                            class="button-side-margins"
                            v-model="vfdSetpoint"
                            variant="primary"
                            v-on:click="damperForward()"
                            >Increase</b-button
                          >
                          <b-button
                            variant="secondary"
                            class="button-side-margins"
                            v-model="vfdSetpoint"
                            v-on:click="damperBack()"
                            >Decrease</b-button
                          >
                        </b-form>
                      </div>

                        <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >Coarse control:
                          </label>
                          <b-button
                            class="button-side-margins"
                            v-model="vfdSetpoint"
                            variant="primary"
                            v-on:click="damperForwardCoarse()"
                            >Increase</b-button
                          >
                          <b-button
                            variant="secondary"
                            class="button-side-margins"
                            v-model="vfdSetpoint"
                            v-on:click="damperBackCoarse()"
                            >Decrease</b-button
                          >
                        </b-form>

                        <br />
                        <p class="description-text"></p>
                      </div>
                    </form>
                  </b-card-text>
                </b-tab>

                <b-tab title="Height Control">
                  <b-card-text>
                    <h4>Height Offset</h4>
                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >Offset:
                          </label>
                          <b-form-input
                            id="inline-form-offset"
                            class="mb-2 mr-sm-2 mb-sm-0 form-input"
                            v-model="heightOffset"
                            @keypress="isNumberNeg($event)"
                          ></b-form-input>

                          <b-button
                            variant="primary"
                            v-on:click="heightOffsetFunc()"
                            >Apply</b-button
                          >
                        </b-form>
                        <br />
                        <p class="description-text">
                          Apply offset to zero the object height.
                        </p>
                      </div>
                    </form>

                    <hr />

                    <h4>PID Controller</h4>

                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >Height:
                          </label>
                          <b-form-input
                            id="inline-form-offset"
                            class="mb-2 mr-sm-2 mb-sm-0 form-input"
                            v-model="height_setpoint"
                            @keypress="isNumber($event)"
                          ></b-form-input>

                          <b-button
                            variant="primary"
                            v-on:click="heightSetpointFunc()"
                            >Apply</b-button
                          >
                        </b-form>

                        <br />
                        <p class="description-text">
                          Setpoint between 0 and 500 (mm).
                        </p>
                      </div>
                    </form>

                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >P Term:
                          </label>
                          <b-form-input
                            id="inline-form-offset"
                            class="mb-2 mr-sm-2 mb-sm-0 form-input"
                            v-model="p_term"
                            @keypress="isNumberNeg($event)"
                          ></b-form-input>

                          <b-button variant="primary" v-on:click="pTermFunc()"
                            >Apply</b-button
                          >
                        </b-form>
                      </div>
                    </form>

                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >I Term:
                          </label>
                          <b-form-input
                            id="inline-form-offset"
                            class="mb-2 mr-sm-2 mb-sm-0 form-input"
                            v-model="i_term"
                            @keypress="isNumberNeg($event)"
                          ></b-form-input>

                          <b-button variant="primary" v-on:click="iTermFunc()"
                            >Apply</b-button
                          >
                        </b-form>
                      </div>
                    </form>

                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >D Term:
                          </label>
                          <b-form-input
                            id="inline-form-offset"
                            class="mb-2 mr-sm-2 mb-sm-0 form-input"
                            v-model="d_term"
                            @keypress="isNumberNeg($event)"
                          ></b-form-input>

                          <b-button variant="primary" v-on:click="dTermFunc()"
                            >Apply</b-button
                          >
                        </b-form>
                      </div>
                    </form>

                    <form>
                      <div class="form-row">
                        <label class="mr-sm-2 bolded" for="inline-form-offset"
                          >PID Controller:
                        </label>
                        <b-form-radio-group
                          id="radio-group-1"
                          v-model="pid_toggle"
                          :options="pid_toggle_options"
                          name="radio-options"
                          inline
                        ></b-form-radio-group>
                        <b-button variant="primary" v-on:click="pidToggleFunc()"
                          >Apply</b-button
                        >
                      </div>
                    </form>
                  </b-card-text>
                </b-tab>

                <b-tab title="System Characteristics">
                  <b-card-text>

                  <hr>
                  <h4>Height Controller</h4>
                    <div>
                      <b-table
                        striped
                        hover
                        stacked
                        :items="heightPidTable"
                        :fields="heightFields"
                      ></b-table>
                    </div>
                  </b-card-text>
                </b-tab>

                <b-tab title="Advanced Settings">
                  <b-card-text>

<hr>

                    <h4>System Tuning</h4>

                    <form>
                      <div class="form-row">
                        <b-form inline>
                          <label class="mr-sm-2 bolded" for="inline-form-offset"
                            >Base RPM:
                          </label>
                          <b-form-input
                            id="inline-form-offset"
                            class="mb-2 mr-sm-2 mb-sm-0 form-input"
                            v-model="base_rpm"
                            @keypress="isNumber($event)"
                          ></b-form-input>

                          <b-button variant="primary" v-on:click="baseRPMFunc()"
                            >Apply</b-button
                          >
                        </b-form>
                      </div>
                    </form>

                    <hr>

                    <h4>Primary Axis</h4>

                    <form>
                      <div class="form-row">
                        <div class="col">
                          <input
                            v-model="primaryAxisMin"
                            @keypress="isNumber($event)"
                            class="form-control"
                            placeholder="minimum"
                          />
                        </div>
                        <div class="col">
                          <input
                            v-model="primaryAxisMax"
                            @keypress="isNumber($event)"
                            class="form-control"
                            placeholder="maximum"
                          />
                        </div>
                      </div>
                    </form>

                    <div class="form-row">
                      <b-button
                        id="change-axis"
                        v-on:click="newAxes()"
                        block
                        variant="primary"
                        class="form-button"
                        >Apply</b-button
                      >
                    </div>
                  </b-card-text>
                </b-tab>
              </b-tabs>
            </b-card>
          </div>

          <!-- <div id="div-3">
            <p>test</p>
          </div> -->
        </div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.form-input {
  max-width: 200px;
}

.button-side-margins {
  margin: 0 2px;
}

.description-text {
  font-size: 16px;
  color: grey;
  margin: 5px 0px;
}

.bolded {
  font-weight: bold;
}
.form-row {
  margin-top: 20px;
  margin-bottom: 20px;
}
#mode-title {
  font-family: 'Courier New', Courier, monospace;
  text-align: left;
}
.axis-select-div {
  padding-right: 0px;
  margin-right: 0px;
}

.axis-select-title {
  font-weight: bold;
  text-align: right;
}

.form-button {
  margin: 20px 5px;
}

#mode {
  text-align: left;
  color: black;
  font-family: 'Courier New', Courier, monospace;
}

.panel-title {
  color: black;
  font-size: x-large;
  font-family: 'Courier New', Courier, monospace;
  padding-top: 8px;
}

.panel-heading {
  background-color: lightseagreen;
  padding-left: 0px;
  padding-right: 0px;
  padding-top: 15px;
  border-bottom: #006100 solid 2px;
}

.container {
  padding-left: 0px;
  padding-right: 0px;
}

#start {
  background-color: lightgreen;
}

#stop {
  background-color: lightcoral;
}

.col2 {
  flex: auto;
  min-width: 250;
  border: 2px solid black;
  border-radius: 15px;
  margin: 10px;
  padding: 0px;
}

.button-col {
  flex: auto;
  max-width: fit-content;
  margin: 10px;
  padding: 0px;
}

.mode-col {
  flex: auto;
  margin: 10px;
  padding: 0px;
}

.panel {
  border: 2px solid #006100;
  border-radius: 5px;
  margin-bottom: 10px;
}

.indicator-box {
  background-color: whitesmoke;
}

#voltage {
  background-color: whitesmoke;
}

#air-speed {
  background-color: whitesmoke;
}

#height {
  background-color: whitesmoke;
}

#speed {
  background-color: #f94c66;
}

h2 {
  font-weight: bolder;
  font-family: 'Courier New', Courier, monospace;
  margin-bottom: 0;
  font-size: 36px;
  margin-top: 5px;
}

h4 {
  font-weight: bolder;
  font-family: 'Courier New', Courier, monospace;
  font-size: 24px;
}

.indicator {
  font-size: 18px;
  font-family: 'Courier New', Courier, monospace;
}

.container-fluid {
  display: flex;
  flex-direction: row;
  justify-content: center;
  height: auto;
  width: 100%;
  padding: 10px;
  flex-grow: 4;
}

.container-fluid-2 {
  display: inline-flex;
  flex-direction: row;
  justify-content: center;
  height: auto;
  width: 100%;
  flex-grow: 1;
  min-width: auto;
}

.container-fluid-3 {
  display: inline-flex;
  flex-direction: column;
  justify-content: center;
}

#div-1 {
  flex-grow: 2;
  margin: 5px;
  padding: 20px;
  border: 2px solid #006100;
  border-radius: 10px;
  margin-right: 20px;
  flex-direction: column;
  display: flex;
  background-color: white;
  min-width: 400px;
}

#div-2 {
  flex-grow: 1;
  margin: 2px;
  padding: 2px;
  border: 2px solid #006100;
  border-radius: 10px;
  margin-left: 20px;
  background-color: white;
  max-width: 500px;
}

.form-row {
  max-width: 400px;
}

#div-3 {
  flex-grow: 1;
  margin: 5px;
  padding: 5px;
  border: 2px solid #006100;
  border-radius: 10px;
  background-color: white;
}

#chart_container {
  padding-right: 120px;

  margin-top: 10px;
  position: relative;
  /* width: 1120px; */
}

.chart-padding {
  padding-top: 20px;
  padding-bottom: 20px;
}

#demo_chart {
  position: relative;
  left: 80px;
}

#y_axis {
  position: absolute;
  top: 0;
  bottom: 0;
  width: 60px;
}

#y_axis_secondary {
  position: absolute;
  top: 0;
  bottom: 0;
  width: 60px;
  right: 20px;
}

#radio-button-choices {
  padding-top: 10px;
}

.button {
  min-width: 150px;
  align-items: center;
  appearance: none;
  background-color: #fcfcfd;
  border-radius: 4px;
  border-width: 0;
  box-shadow: rgba(45, 35, 66, 0.4) 0 2px 4px,
    rgba(45, 35, 66, 0.3) 0 7px 13px -3px, #d6d6e7 0 -3px 0 inset;
  box-sizing: border-box;
  color: #36395a;
  cursor: pointer;
  display: inline-flex;
  font-family: 'JetBrains Mono', monospace;
  height: 48px;
  justify-content: center;
  line-height: 1;
  list-style: none;
  overflow: hidden;
  padding-left: 16px;
  padding-right: 16px;
  position: relative;
  text-align: left;
  text-decoration: none;
  transition: box-shadow 0.15s, transform 0.15s;
  user-select: none;
  -webkit-user-select: none;
  touch-action: manipulation;
  white-space: nowrap;
  will-change: box-shadow, transform;
  font-size: 24px;
  font-weight: bold;
}

.button:focus {
  box-shadow: #d6d6e7 0 0 0 1.5px inset, rgba(45, 35, 66, 0.4) 0 2px 4px,
    rgba(45, 35, 66, 0.3) 0 7px 13px -3px, #d6d6e7 0 -3px 0 inset;
}

.button:hover {
  box-shadow: rgba(45, 35, 66, 0.4) 0 4px 8px,
    rgba(45, 35, 66, 0.3) 0 7px 13px -3px, #d6d6e7 0 -3px 0 inset;
  transform: translateY(-2px);
}

.button:active {
  box-shadow: #d6d6e7 0 3px 7px inset;
  transform: translateY(2px);
}
</style>

<script>
import io from 'socket.io-client'
import Rickshaw from 'rickshaw'
import 'rickshaw/rickshaw.min.css'
import 'bootstrap/dist/css/bootstrap.css'
// import 'bootstrap/dist/js/bootstrap.js'
import 'bootstrap-vue/dist/bootstrap-vue.css'
import 'bootstrap-vue/dist/bootstrap-vue.js'
import { ref } from 'vue'

var socket = io.connect('http://192.168.1.85:3000')
// var socket_ = io();

var magnitudeChart
var y_axis
var y_axis_secondary

export default {
  name: 'home',
  data () {
    return {
      renderEveryNth: 5,
      updateInterval: 20,
      streamFrequency: 10,
      connStatus: 'Disconnected',
      messageIndex: {
        voltage: 0,
        motorSpeed: 0,
        airflowSpeed: 0,
        objectHeight: 0
      },
      messageSeries: {
        voltage: [],
        motorSpeed: [],
        airflowSpeed: [],
        objectHeight: [],
        none: []
      },
      displayedValues: {
        voltage: [],
        motorSpeed: [],
        airflowSpeed: [],
        objectHeight: []
      },
      dvColors: {
        objectHeight: '#cb503a',
        airflowSpeed: '#72c039',
        voltage: '#65b9ac'
      },
      primarySelected: 'motorSpeed',
      // secondarySelected: 'none',
      options: [
        { text: ' Motor Speed', value: 'motorSpeed' },
        { text: ' Airflow Speed', value: 'airflowSpeed' },
        { text: ' Ball Height', value: 'objectHeight' },
        { text: ' None', value: 'none' }
      ],
      optionsArray: [
        'motorSpeed',
        'airflowSpeed',
        'objectHeight',
        'voltage',
        'none'
      ],
      primaryAxisMin: 600,
      primaryAxisMax: 1850,
      // secondaryAxisMin: 0,
      // secondaryAxisMax: 300,
      mqttVoltageTopic: 'voltage',
      mqttAirTopic: 'airflowSpeed',
      mqttHeightTopic: 'objectHeight',
      mqttmotorSpeedTopic: 'motorSpeed',
      mode: 'Manual Control',
      heightOffset: '0',
      vfdSetpoint: '800',
      pid_toggle: '0',
      pid_toggle_options: [
        { text: 'Off', value: '0' },
        { text: 'On', value: '1' }
      ],
      p_term: '0.5',
      i_term: '0.012',
      d_term: '250',
      height_axis_min: 0,
      height_axis_max: 600,
      rpm_axis_min: 600,
      rpm_axis_max: 1850,
      airflow_axis_min: 0,
      airflow_axis_max: 40,
      height_setpoint: '0',
      base_rpm: '1200',
      mqttSystemHeightTopic: 'system_property_JSON',
      system_height_p: 0,
      system_height_i: 0,
      system_height_d: 0,
      system_height_base_rpm: 0,
      system_height_setpoint: 0,
      heightFields: [
        {
          key: 'setpoint',
          label: 'Height Setpoint'
        },
        {
          key: 'base',
          label: 'Base RPM'
        },
        {
          key: 'p_term',
          label: 'P Gain'
        },
        {
          key: 'i_term',
          label: 'I Gain'
        },
        {
          key: 'd_term',
          label: 'D Gain'
        }
        
      ],
      motorSpeedSeries: {
        motorSpeed: []
      },
      objectHeightSeries: {
        objectHeight: []
      },
      airflowSpeedSeries: {
        airflowSpeed: []
      }
    }
  },

  computed: {
    // a computed getter
    heightPidTable () {
      let items = [
        {
          p_term: this.system_height_p,
          i_term: this.system_height_i,
          d_term: this.system_height_d,
          base: this.system_height_base_rpm,
          setpoint: this.system_height_setpoint
        }
      ]

      return items
    }
  },

  mounted () {
    this.initChart()
    this.openSocketListeners()
  },
  watch: {
    renderEveryNth: function () {
      this.messageSeries = {
        voltage: [],
        motorSpeed: [],
        airflowSpeed: [],
        objectHeight: []
      }
      this.messageIndex = {
        voltage: 0,
        motorSpeed: 0,
        airflowSpeed: 0,
        objectHeight: 0
      }

      this.motorSpeedSeries= {
        motorSpeed: []
      },
      this.objectHeightSeries = {
        objectHeight: []
      },
      this.airflowSpeedSeries = {
        airflowSpeed: []
      }
    }
  },
  methods: {
    motorStart () {
      socket.emit('rpmSetpoint', '800')
      socket.emit('motor', '1')
      socket.emit('clear_error', '1')
    },

    motorStop () {
      socket.emit('motor', '0')
      socket.emit('clear_error', '1')
    },

    isNumber (evt) {
      const charCode = evt.which ? evt.which : evt.keyCode
      if (
        charCode > 31 &&
        (charCode < 48 || charCode > 57) &&
        charCode !== 46
      ) {
        evt.preventDefault()
      }
    },

    heightOffsetFunc () {
      socket.emit('offset', this.heightOffset)
    },

    isNumberNeg (evt) {
      const charCode = evt.which ? evt.which : evt.keyCode
      if (
        charCode > 31 &&
        (charCode < 48 || charCode > 57) &&
        charCode !== 46 &&
        charCode !== 45
      ) {
        evt.preventDefault()
      }
    },

    heightSetpointFunc () {
      socket.emit('height_setpoint', this.height_setpoint)
    },

    pTermFunc () {
      socket.emit('p_term', this.p_term)
    },

    iTermFunc () {
      socket.emit('i_term', this.i_term)
    },

    dTermFunc () {
      socket.emit('d_term', this.d_term)
    },

    pidToggleFunc () {
      socket.emit('pid_toggle', this.pid_toggle)
      socket.emit('clear_error', '1')

      if (this.pid_toggle === '0') {
        this.mode = 'Manual Control'
      } else {
        this.mode = 'PID Control'
      }
    },

    setPointRequestFunc () {
      socket.emit('rpmSetpoint', this.vfdSetpoint)
    },

    setPointCoarseIncreaseFunc () {
      socket.emit('rpmMod', '50')
    },

    setPointCoarseDecreaseFunc () {
      socket.emit('rpmMod', '-50')
    },

    setPointFineIncreaseFunc () {
      socket.emit('rpmMod', '5')
    },

    setPointFineDecreaseFunc () {
      socket.emit('rpmMod', '-5')
    },

    clearErrorFunc () {
      socket.emit('clear_error', '1')
    },

    baseRPMFunc () {
      socket.emit('base_rpm', this.base_rpm)
    },

    damperForward () {
      socket.emit('damperForward', "1")
    },

    damperBack () {
      socket.emit('damperBack', "2")
    },

    damperForwardCoarse () {
      socket.emit('damperForwardCoarse', "3")
    },

    damperBackCoarse () {
      socket.emit('damperBackCoarse', "4")
    },

    /* Rickshaw.js initialization */
    initChart () {
      magnitudeChart = new Rickshaw.Graph({
        element: document.querySelector('#demo_chart'),
        width: '100',
        height: '420',
        renderer: 'line',
        padding: { top: 0.02, left: 0.02, right: 0.02, bottom: 0.02 },
        margin: { top: 0.02, left: 0.02, right: 0.02, bottom: 0.02 },
        min: this.primaryAxisMin,
        max: this.primaryAxisMax,
        series: new Rickshaw.Series.FixedDuration(
          [
            {
              name: 'series1',
              color: '#dd5511'
            }
          ],
          undefined,
          {
            timeInterval: this.updateInterval,
            maxDataPoints: 500,
            timeBase: new Date().getTime() / 1000
          }
        )
      })

      y_axis = new Rickshaw.Graph.Axis.Y({
        graph: magnitudeChart,
        orientation: 'left',
        tickFormat: function (y) {
          return y.toFixed(1)
        },
        ticks: 8,
        element: document.getElementById('y_axis')
      })

      // y_axis_secondary = new Rickshaw.Graph.Axis.Y({
      //   graph: magnitudeChart,
      //   orientation: 'left',
      //   tickFormat: function (y) {
      //     return y.toFixed(1)
      //   },
      //   ticks: 8,
      //   element: document.getElementById('y_axis_secondary')
      // })

      this.resizeChart(magnitudeChart)

      window.addEventListener('resize', () => {
        this.resizeChart(magnitudeChart)
      })

      document.getElementById('change-axis').addEventListener('click', () => {
        this.resizeChart(magnitudeChart)
      })

      document
        .getElementById('y--axis-select')
        .addEventListener('click', () => {
          this.newAxesRadio(magnitudeChart)
        })
    },

    resizeChart (chart) {
      chart.configure({
        width: this.$refs.panel.clientWidth,
        min: this.primaryAxisMin,
        max: this.primaryAxisMax
      })

      chart.render()
    },

    newAxes () {
      this.magnitudeChart.configure({
        min: this.primaryAxisMin,
        max: this.primaryAxisMax
      })
      chart.render()
    },

    newAxesRadio (chart) {
      let axis_min = 0
      let axis_max = 0

      if (this.primarySelected === 'objectHeight') {
        axis_min = this.height_axis_min
        axis_max = this.height_axis_max
      } 
      else if (this.primarySelected === 'motorSpeed') {
        axis_min = this.rpm_axis_min
        axis_max = this.rpm_axis_max
      }
      else if (this.primarySelected === 'airflowSpeed') {
        axis_min = this.airflow_axis_min
        axis_max = this.airflow_axis_max
      }
      chart.configure({
        min: axis_min,
        max: axis_max
      })
      chart.render()
    },

    /* Insert received datapoints into the chart */
    insertDatapoints (messageSeries, chart, topic, renderFreq) {     

        var message
        for (let i = 0; i < renderFreq; i++) {
          message = messageSeries[topic][i]
          let data = {
            topic: message,
            setpoint: this.system_height_setpoint
          }
          console.log("Adding data: " + this.primarySelected + ", " + data[topic])
          chart.series.addData(data)
        }

        chart.render()
      
    },

    onPrimaryChange (event) {
      const boxes = document.querySelectorAll('.indicator-box')

      boxes.forEach(box => {
        box.style.backgroundColor = 'whitesmoke'
      })

      switch (this.primarySelected) {
        case 'motorSpeed':
          var motorSpeed = document.getElementById('speed')
          motorSpeed.style.backgroundColor = 'rgb(249, 76, 102)'
          break
        case 'voltage':
          var voltage = document.getElementById('voltage')
          voltage.style.backgroundColor = 'rgb(249, 76, 102)'
          break

        case 'objectHeight':
          var height = document.getElementById('height')
          height.style.backgroundColor = 'rgb(249, 76, 102)'

          break

        case 'airflowSpeed':
          var airSpeed = document.getElementById('air-speed')
          airSpeed.style.backgroundColor = 'rgb(249, 76, 102)'
          break
      }
    },

    /* Update displayed values*/
    updateDisplayedValues (type, messageSeries) {
      if (this.messageIndex[type] == this.streamFrequency) {
        this.messageIndex[type] = 0
        this.displayedValues[type] = messageSeries[type]
        this.messageIndex[type]++
      } else {
        this.messageIndex[type]++
      }
    },
    openSocketListeners () {
      socket.on('connect', () => {
        this.connStatus = 'Connected'
      })

      socket.on('disconnect', () => {
        this.connStatus = 'Disconnected'
      })

      socket.on(this.mqttSystemHeightTopic, message => {
        this.system_height_p = message.p_term
        this.system_height_i = message.i_term
        this.system_height_d = message.d_term
        this.system_height_base_rpm = message.base_rpm
        this.system_height_setpoint = message.height_setpoint
      })

      for (let topic of this.optionsArray) {
        socket.on(topic, message => {
          var type = message.type
          //console.log(message.type)
          var messageSeries

          switch(topic) {
            case "motorSpeed":
              messageSeries = this.motorSpeedSeries
              break;
            case "objectHeight":
              messageSeries = this.objectHeightSeries
              break;
              case "airflowSpeed":
              messageSeries = this.airflowSpeedSeries
              break;
          }

          

          /* Push stream data to current series, if it's not yet render-time */
          if (messageSeries[topic].length < this.renderEveryNth) {
            messageSeries[type].push(message.data)

            /* Check if displayed values have to be updated */
            this.updateDisplayedValues(type, messageSeries)
            //console.log(messageSeries);
          }

          /* Render-time! */
          if (this.primarySelected == topic && messageSeries[type].length == this.renderEveryNth) {
            this.insertDatapoints(
              messageSeries,
              magnitudeChart,
              topic,
              this.renderEveryNth
            )
            this.motorSpeedSeries["motorSpeed"] = []
            this.objectHeightSeries["objectHeight"] = []
            this.airflowSpeedSeries["airflowSpeed"] = []
          }
        })
      }
    }
  }
}
</script>
