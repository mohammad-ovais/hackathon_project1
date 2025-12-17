# Quickstart Guide: Physical AI Robotics Textbook

## Prerequisites

- Node.js 18+ installed
- Git installed
- Basic knowledge of JavaScript/TypeScript

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Install dependencies**
   ```bash
   cd docusaurus
   npm install
   ```

3. **Start the development server**
   ```bash
   npm start
   ```

4. **Open your browser**
   Navigate to `http://localhost:3000` to view the textbook

## Building for Production

```bash
npm run build
```

The built site will be available in the `build/` directory and can be deployed to GitHub Pages.

## Deployment to GitHub Pages

1. Configure `docusaurus.config.js` with your GitHub Pages settings
2. Run the deployment command:
   ```bash
   GIT_USER=<your-github-username> npm run deploy
   ```

## Project Structure

- `docs/` - Contains all textbook content organized by modules
- `src/` - Custom React components and pages
- `static/` - Static assets like images and files
- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration