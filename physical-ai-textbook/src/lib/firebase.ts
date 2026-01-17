import { initializeApp, getApps, FirebaseApp } from "firebase/app";
import { getAuth, Auth } from "firebase/auth";
import { getFirestore, Firestore } from "firebase/firestore";

// Get Firebase config from Docusaurus customFields
const getFirebaseConfig = () => {
  if (typeof window !== "undefined") {
    // Access Docusaurus site config on client side
    const siteConfig = (window as any).__DOCUSAURUS__;
    if (siteConfig?.siteConfig?.customFields?.firebase) {
      return siteConfig.siteConfig.customFields.firebase;
    }
  }

  // Fallback config (will be replaced with real values in production)
  return {
    apiKey: "AIzaSyDemo-placeholder-key",
    authDomain: "demo-project.firebaseapp.com",
    projectId: "demo-project",
    storageBucket: "demo-project.appspot.com",
    messagingSenderId: "123456789",
    appId: "1:123456789:web:abcdef",
  };
};

// Initialize Firebase only on client side and only once
let app: FirebaseApp | undefined;
let auth: Auth | undefined;
let db: Firestore | undefined;

export const initializeFirebase = () => {
  if (typeof window !== "undefined" && !app) {
    const firebaseConfig = getFirebaseConfig();

    if (!getApps().length) {
      app = initializeApp(firebaseConfig);
    } else {
      app = getApps()[0];
    }
    auth = getAuth(app);
    db = getFirestore(app);
  }
  return { app, auth, db };
};

// Auto-initialize on import (client-side only)
if (typeof window !== "undefined") {
  initializeFirebase();
}

export { app, auth, db };
export default app;
