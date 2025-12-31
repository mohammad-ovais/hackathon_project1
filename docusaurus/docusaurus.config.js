// @ts-check
// `@type` JSDoc annotations allow IDEs and type-checking tools to autocomplete
// and validate function arguments and return values, enhancing code quality.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Robotics: From Foundations to Advanced Applications',
  tagline: 'A comprehensive textbook on robotics, AI, and autonomous systems',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://mohammad-ovais.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this is usually '/<organization-name>/<repository-name>/'
  baseUrl: '/hackathon_project1/',
  scripts: [
    {
      src: '/chat-widget.js',
      async: true,
      'data-backend-url': 'https://ovais123-rag-chatbot.hf.space',
      'data-theme': 'light',
      'data-position': 'right',
    },
  ],
  // GitHub pages deployment config.
  organizationName: 'mohammad-ovais', // Usually your GitHub org/user name.
  projectName: 'hackathon_project1', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/physical-ai-robotics-book/physical-ai-robotics-book/tree/main/docusaurus/',
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  plugins: [
    [
      './src/plugins/chatbot-plugin.js',
      {
        backendUrl: 'https://ovais123-rag-chatbot.hf.space', // Update this to your backend URL
        theme: 'light',
        position: 'right'
      }
    ]
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Robotics Textbook',
        logo: {
          alt: 'Physical AI Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/physical-ai-robotics-book/physical-ai-robotics-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Module 1: The Robotic Nervous System (ROS 2)',
                to: '/docs/module-1/intro',
              },
              {
                label: 'Module 2: The Digital Twin (Gazebo & Unity)',
                to: '/docs/module-2/intro',
              },
              {
                label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
                to: '/docs/module-3/intro',
              },
              {
                label: 'Module 4: Vision-Language-Action (VLA)',
                to: '/docs/module-4/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/physical-ai-robotics-book/physical-ai-robotics-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;
