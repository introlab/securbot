<template>
  <div />
</template>

<script>
/**
 * Element to be implemented in future release.
 * Component used as a page to log the communications and interactions between the operator and
 * the robot. It will show the event logged in a database (component), allow the user to filter
 * through the events (component) and query the database (Events.vue will manage those)
 * There are currently no components in this page. This component have the following
 * dependency : Bootstrap-Vue for styling.
 *
 *
 * @module Events
 * @vue-prop {Vue} bus - Vue bus use to emit event to other components.
 * @vue-prop {Vue} Router - Vue bus use to routing emit event to parent.
 * @vue-event {} destroyed - Event indicating the component has been destroyed.
 * @vue-event {} mounted - Event indicating the component has been mounted.
 */

/* Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import Vue from 'vue';

export default {
  name: 'event-page',
  components: {
  },
  props: {
    bus: {
      type: Vue,
      required: true,
    },
    router: {
      type: Vue,
      required: true,
    },
  },
  /**
   * Lifecycle Hook - mounted
   *
   * @method
   * @listens mount(el)
   */
  mounted() {
    console.log('Event have been mounted');
    this.bus.$emit('mounted');
  },
  /**
   * Lifecycle Hook - mounted
   *
   * @method
   * @listens destroyed(el)
   */
  destroyed() {
    console.log('Event have been destroyed');
    this.router.$emit('destroyed');
  },
  methods: {
  },
};
</script>

<style>

</style>
