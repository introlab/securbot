<template>
  <div>
    <!-- Navbar container-->
    <div
      id="sec-nav"
      class="position-relative"
    >
      <!-- Navbar -->
      <b-navbar
        toggleable="md"
        class="navbar-dark mb-0 bg-green-sb"
      >
        <!-- Brand -->
        <b-navbar-brand class="p-0">
          <div
            class="h-100"
            style="width:240px"
          >
            <img
              src="../../assets/SecurBotLogo.png"
              alt="SecurBot"
              class="logo mh-100 mw-100 align-middle"
            >
          </div>
        </b-navbar-brand>
        <!-- Collapse Option -->
        <b-navbar-toggle target="nav_collapse" />
        <!-- Collapsable Navbar -->
        <b-collapse
          id="nav_collapse"
          is-nav
        >
          <!-- Navbar right side content -->
          <b-navbar-nav>
            <!-- Teleop -->
            <b-nav-item to="teleop">
              Teleoperation
            </b-nav-item>
            <!-- Patrol -->
            <b-nav-item to="patrol">
              Patrol Planner
            </b-nav-item>
            <!-- Event -->
            <b-nav-item to="logs">
              Logs
            </b-nav-item>
            <b-nav-item to="testing">
              .
            </b-nav-item>
          </b-navbar-nav>
          <!-- Navbar left side content -->
          <b-navbar-nav class="ml-auto">
            <!-- Dropdown with connection widget -->
            <b-nav-item-dropdown
              text="Connect to Robot"
              right
            >
              <!-- Connection container -->
              <div class="px-2 py-1">
                <!-- Connection -->
                <connection
                  :self-id="myId"
                  :robot-list="robotList"
                  :bus="teleopBus"
                />
              </div>
            </b-nav-item-dropdown>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>
  </div>
</template>

<script>
import Vue from 'vue';
import Connection from './Connection';

export default {
  name: 'navbar',
  components: {
    Connection,
  },
  props: {
    myId: {
      type: String,
      required: true,
    },
    robotList: {
      type: Array,
      required: true,
    },
    teleopBus: {
      type: Vue,
      required: true,
    },
  },
};
</script>

<style lang="css" scoped>
.navbar{
  min-height:64px;
}
/* #sec-nav a {
  font-weight: bold;
  color: white;
} */
#sec-nav a.router-link-exact-active {
  color: white;
}
</style>
