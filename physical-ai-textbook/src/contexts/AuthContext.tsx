import React, {
  createContext,
  useContext,
  useEffect,
  useState,
  ReactNode,
} from "react";
import {
  User,
  signInWithEmailAndPassword,
  createUserWithEmailAndPassword,
  signOut as firebaseSignOut,
  onAuthStateChanged,
  GoogleAuthProvider,
  signInWithPopup,
  sendPasswordResetEmail,
  updateProfile,
} from "firebase/auth";
import { auth, initializeFirebase } from "../lib/firebase";

interface AuthContextType {
  user: User | null;
  loading: boolean;
  error: string | null;
  signInWithEmail: (email: string, password: string) => Promise<void>;
  signUpWithEmail: (
    email: string,
    password: string,
    displayName: string,
  ) => Promise<void>;
  signInWithGoogle: () => Promise<void>;
  signOut: () => Promise<void>;
  resetPassword: (email: string) => Promise<void>;
  clearError: () => void;
  getIdToken: () => Promise<string | null>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
}

interface AuthProviderProps {
  children: ReactNode;
}

// Mock user for when Firebase is not configured
const MOCK_USER = {
  uid: "anonymous-user",
  email: "anonymous@local.dev",
  displayName: "Anonymous User",
  getIdToken: async () => "mock-token",
} as unknown as User;

export function AuthProvider({
  children,
}: AuthProviderProps): React.ReactElement {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    // Initialize Firebase on mount
    initializeFirebase();

    if (!auth) {
      // Firebase not configured - use mock user for local development
      console.log("Firebase not configured - using anonymous mode");
      setUser(MOCK_USER);
      setLoading(false);
      return;
    }

    const unsubscribe = onAuthStateChanged(auth, (user) => {
      setUser(user);
      setLoading(false);
    });

    return () => unsubscribe();
  }, []);

  const clearError = () => setError(null);

  const signInWithEmail = async (
    email: string,
    password: string,
  ): Promise<void> => {
    if (!auth) {
      setError("Authentication not initialized");
      return;
    }

    try {
      setError(null);
      await signInWithEmailAndPassword(auth, email, password);
    } catch (err: any) {
      const message = getErrorMessage(err.code);
      setError(message);
      throw err;
    }
  };

  const signUpWithEmail = async (
    email: string,
    password: string,
    displayName: string,
  ): Promise<void> => {
    if (!auth) {
      setError("Authentication not initialized");
      return;
    }

    try {
      setError(null);
      const result = await createUserWithEmailAndPassword(
        auth,
        email,
        password,
      );

      // Update display name
      if (result.user) {
        await updateProfile(result.user, { displayName });
      }
    } catch (err: any) {
      const message = getErrorMessage(err.code);
      setError(message);
      throw err;
    }
  };

  const signInWithGoogle = async (): Promise<void> => {
    if (!auth) {
      setError("Authentication not initialized");
      return;
    }

    try {
      setError(null);
      const provider = new GoogleAuthProvider();
      await signInWithPopup(auth, provider);
    } catch (err: any) {
      const message = getErrorMessage(err.code);
      setError(message);
      throw err;
    }
  };

  const signOut = async (): Promise<void> => {
    if (!auth) {
      setError("Authentication not initialized");
      return;
    }

    try {
      setError(null);
      await firebaseSignOut(auth);
    } catch (err: any) {
      const message = getErrorMessage(err.code);
      setError(message);
      throw err;
    }
  };

  const resetPassword = async (email: string): Promise<void> => {
    if (!auth) {
      setError("Authentication not initialized");
      return;
    }

    try {
      setError(null);
      await sendPasswordResetEmail(auth, email);
    } catch (err: any) {
      const message = getErrorMessage(err.code);
      setError(message);
      throw err;
    }
  };

  const getIdToken = async (): Promise<string | null> => {
    if (user?.getIdToken) {
      return await user.getIdToken();
    }
    return "mock-token";
  };

  const value: AuthContextType = {
    user,
    loading,
    error,
    signInWithEmail,
    signUpWithEmail,
    signInWithGoogle,
    signOut,
    resetPassword,
    clearError,
    getIdToken,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

// Helper function to get user-friendly error messages
function getErrorMessage(errorCode: string): string {
  switch (errorCode) {
    case "auth/email-already-in-use":
      return "This email is already registered. Please sign in instead.";
    case "auth/invalid-email":
      return "Invalid email address format.";
    case "auth/operation-not-allowed":
      return "This sign-in method is not enabled.";
    case "auth/weak-password":
      return "Password is too weak. Please use at least 6 characters.";
    case "auth/user-disabled":
      return "This account has been disabled.";
    case "auth/user-not-found":
      return "No account found with this email.";
    case "auth/wrong-password":
      return "Incorrect password.";
    case "auth/invalid-credential":
      return "Invalid email or password.";
    case "auth/too-many-requests":
      return "Too many failed attempts. Please try again later.";
    case "auth/popup-closed-by-user":
      return "Sign-in popup was closed. Please try again.";
    case "auth/network-request-failed":
      return "Network error. Please check your connection.";
    default:
      return "An error occurred. Please try again.";
  }
}

export default AuthContext;
