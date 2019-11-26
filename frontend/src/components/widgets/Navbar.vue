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
            <b-nav-item to="patrol-planner">
              Patrol Planner
            </b-nav-item>
            <b-nav-item to="patrol-config">
              Patrol Config
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
            <div
              style="font-size: 0.8rem; max-height: calc( 64px - 2rem )"
            >
              <transition name="nav-fade">
                <robot-status />
              </transition>
            </div>
            <!-- Dropdown with connection widget -->
            <b-nav-item-dropdown
              :text="(isConnected ? `Connected to ${robotIdToName(robotId)}` : 'Connect to Robot')"
              right
            >
              <!-- Connection container -->
              <div class="px-2 py-1">
                <!-- Connection -->
                <transition name="conn-fade">
                  <interactive-list
                    :list="robotList"
                    display-key="robotName"
                    @click="handleConnection"
                  >
                    <template v-slot:header>
                      <div class="d-flex flex-row justify-content-between">
                        <h5 class="ml-1 mb-0 mt-0">
                          List of Robots:
                        </h5>
                        <b-button
                          id="disconnect-all-button"
                          class="m-0"
                          size="sm"
                          style="max-height: 30px"
                          variant="outline-danger"
                          @click="disconnectAll"
                        >
                          <font-awesome-icon
                            icon="power-off"
                          />
                        </b-button>
                        <b-tooltip
                          target="disconnect-all-button"
                          placement="left"
                          variant="secondary"
                        >
                          Disconnect
                        </b-tooltip>
                      </div>
                    </template>
                    <template v-slot:tag="robot">
                      <transition
                        name="status-fade"
                        mode="out-in"
                      >
                        <span
                          v-if="robot.item.robotId === robotId && connectionState === 'connecting'"
                          :key="0"
                          class="spinner-border spinner-border-sm text-warning"
                        />
                        <span
                          v-else-if="robot.item.robotId === robotId"
                          :key="1"
                          class="badge badge-success"
                        >
                          Connected
                        </span>
                        <span
                          v-else
                          :key="2"
                          class="badge badge-secondary"
                        >
                          Not Connected
                        </span>
                      </transition>
                    </template>
                  </interactive-list>
                </transition>
              </div>
            </b-nav-item-dropdown>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex';
import InteractiveList from '../custom/InteractiveList';
import RobotStatus from './RobotStatus';

/**
 * The navigation bar used to change views.
 *
 * Authors:
 *
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>
 *
 * @since 0.1.0
 * @version 1.0.0
 * @displayName Navbar
 */
export default {
  name: 'navbar',
  components: {
    InteractiveList,
    RobotStatus,
  },
  computed: mapState({
    myId: state => state.client.myId,
    robotId: state => state.client.robotId,
    robotList: state => state.client.robotList,
    connectionState: state => state.client.connectionState.robot,
    isConnected: state => state.client.connectionState.robot === 'connected',
  }),
  methods: {
    /**
     * When the interactive list is clicked, this method handles the connection state.
     *
     * @public
     * @param {Object} robot The robot object that includes its id and name.
     * @since 1.1.0
     * @deprecated Will soon be deprecated in profit of a store action.
     */
    handleConnection(robot) {
      const { robotId } = robot;
      if (this.isConnected && robotId === this.robotId) {
        console.log('Disconnecting...');
        this.$store.commit('setConnectedRobot', { robotName: '', robotId: '' });
        this.$store.commit('disableJoystick');
        this.$store.dispatch('client/disconnectFromRobot');
      } else if (this.isConnected && this.robotId) {
        console.log('Already connected to someone...');
      } else if (this.connectionState === 'connecting') {
        console.log('Waiting for state...');
      } else {
        this.$store.commit('setConnectedRobot', robot);
        this.$store.dispatch('client/connectToRobot', robotId);
      }
    },
    disconnectAll() {
      console.log('Disconnecting...');
      this.$store.commit('setConnectedRobot', { robotName: '', robotId: '' });
      this.$store.commit('disableJoystick');
      this.$store.dispatch('client/disconnectFromRobot');
    },
    robotIdToName(robotId) {
      for (const robot of this.robotList) {
        if (robot.robotId === robotId) {
          return robot.robotName;
        }
      }
      return '';
    },
  },
};
</script>

<style>
.conn-fade-enter-active, .conn-fade-leave-active {
  transition: transform 2s ease-in-out;
}
.conn-fade-enter, .conn-fade-leave-to {
  opacity: 0;
}
.status-fade-enter-active, .status-fade-leave-active {
  transition: opacity 0.5s;
}
.status-fade-enter, .status-fade-leave-to {
  opacity: 0;
}
.nav-fade-enter-active, .nav-fade-leave-active {
  transition: opacity 1s;
}
.nav-fade-enter, .nav-fade-leave-to {
  opacity: 0;
}
.navbar{
  min-height: 64px;
}
#sec-nav a.router-link-exact-active {
  color: white;
}
</style>
