const path = require('path');

module.exports = {
	// set your styleguidist configuration here
	title: 'Securbot Frontend Documentation',
	simpleEditor: false,
	ignore: ['**/components/views/Testing.vue', '**/components/views/Testing2.vue'],
	components: 'src/components/**/[A-Z]*.vue',
	defaultExample: false,
	renderRootJsx: path.join(__dirname, 'config/styleguide.root.js'),
	styleguideDir: '../docs/frontend',
	usageMode: 'expand',
	pagePerSection: true,
	sections: [
		{
	    name: 'Introduction',
	    content: 'README.md'
		},
		{
	    name: 'UI Components',
			sections: [
				{
					name: 'Views',
					components: 'src/components/views/[A-Z]*.vue'
				},
				{
					name: 'Widget',
					components: 'src/components/widgets/[A-Z]*.vue'
				},
			],
			sectionDepth: 3
		},
	],
	ribbon: {
    url: 'https://github.com/introlab/securbot/wiki',
    text: 'Check the wiki'
  },
	// webpackConfig: {
	//   // custom config goes here
	// }
	theme: {
		maxWidth: '90%',
		sidebarWidth: 250,
		color: {
			// base: '#ffffff',
			// light: '#f1f1f1',
			linkHover: '#3acf38',
			sidebarBackground: '#00A759',
			errorBackground: '#e22d44',
			// baseBackground: '#222222',
			// codeBackground: '#313131',
			ribbonBackground: '#00A759',
		}
	},
	styles: {
		StyleGuide: {
				root: {
						'text-rendering': 'optimizeLegibility',
						'-moz-osx-font-smoothing': 'grayscale',
						'-webkit-font-smoothing': 'antialiased'
				},
				sidebar: {},
				content: {},
				logo: {
						border: 'none',
						paddingBottom: 0
				}
		},
		Logo: {
				logo: {
						color: '#fff',
						fontSize: 20
				}
		},
		ComponentsList: {
				item: {
						'& a': {
								'color': 'rgba(255, 255, 255, 0.9) !important',
								'fontWeight': 500,
								'&:hover': {
										textDecoration: 'underline',
										color: '#fff !important'
								}
						}
				},
				heading: {
						fontSize: '14px !important',
						fontWeight: '600 !important',
						color: '#fff !important',
				}
		}
	}
}
