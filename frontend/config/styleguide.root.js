import store from '../src/store';

export default previewComponent => (
  {
    store,
    render(createElement) {
      return createElement(previewComponent);
    },
  }
);
