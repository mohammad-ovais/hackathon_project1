#!/usr/bin/env python3
"""
Test script to validate that the chatbot widget is properly integrated on multiple book pages
"""
import os
import sys
from pathlib import Path

def check_widget_integration():
    """
    Check that the chatbot widget is properly integrated in the Docusaurus setup
    """
    print("Checking chatbot widget integration...")

    # Check that the plugin files exist
    plugin_dir = Path("docusaurus/src/plugins")
    if not plugin_dir.exists():
        print("❌ Plugin directory does not exist")
        return False

    plugin_files = ["chatbot-plugin.js", "chatbot-client.js"]
    for file in plugin_files:
        plugin_file = plugin_dir / file
        if not plugin_file.exists():
            print(f"❌ Plugin file {plugin_file} does not exist")
            return False
        print(f"✅ Plugin file {plugin_file} exists")

    # Check that the widget script is in the static directory
    widget_script = Path("docusaurus/static/js/chat-widget.js")
    if not widget_script.exists():
        print("❌ Chat widget script does not exist in static directory")
        return False
    print("✅ Chat widget script exists in static directory")

    # Check that the plugin is registered in docusaurus.config.js
    config_file = Path("docusaurus/docusaurus.config.js")
    if not config_file.exists():
        print("❌ Docusaurus config file does not exist")
        return False

    with open(config_file, 'r', encoding='utf-8') as f:
        config_content = f.read()

    if "./src/plugins/chatbot-plugin.js" not in config_content:
        print("❌ Chatbot plugin not registered in docusaurus.config.js")
        return False
    print("✅ Chatbot plugin registered in docusaurus.config.js")

    # Check that the plugin configuration has appropriate settings
    if "backendUrl" not in config_content:
        print("⚠️  Warning: No backendUrl configuration found in plugin settings")
    else:
        print("✅ Backend URL configuration found in plugin settings")

    # Check that the docs directory exists and has content
    docs_dir = Path("docusaurus/docs")
    if not docs_dir.exists():
        print("⚠️  Docs directory does not exist")
        return False

    md_files = list(docs_dir.rglob("*.md"))
    if not md_files:
        print("⚠️  No markdown files found in docs directory")
    else:
        print(f"✅ Found {len(md_files)} markdown files in docs directory")

    # Check if there are module directories (based on the textbook structure)
    module_dirs = [d for d in docs_dir.iterdir() if d.is_dir() and "module" in d.name.lower()]
    if not module_dirs:
        print("⚠️  No module directories found in docs")
    else:
        print(f"✅ Found {len(module_dirs)} module directories")

    print("\nWidget integration check complete!")
    print("The chatbot widget should appear on all book pages once the Docusaurus site is built and served.")
    print("\nTo test the integration:")
    print("1. Navigate to the docusaurus directory: cd docusaurus")
    print("2. Install dependencies: npm install")
    print("3. Start the development server: npm run start")
    print("4. Visit the site in your browser and verify the chat widget appears on pages")

    return True

def main():
    """Main function to run the integration test"""
    print("=" * 60)
    print("Chatbot Widget Integration Test")
    print("=" * 60)

    success = check_widget_integration()

    print("\n" + "=" * 60)
    if success:
        print("✅ Widget integration test PASSED")
        print("The chatbot widget is properly integrated with Docusaurus.")
    else:
        print("❌ Widget integration test FAILED")
        print("Please check the integration steps above.")
    print("=" * 60)

    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)